#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import pygame
import rospy
import rospkg
from std_msgs.msg import String


class MeterAudioNode:
    def __init__(self):
        rospy.init_node("meter_audio_node", anonymous=True)

        default_audio_dir = os.path.join(
            rospkg.RosPack().get_path("dog_motion"), "audio"
        )
        self.audio_dir = rospy.get_param("~audio_dir", default_audio_dir)
        self.status_topic = rospy.get_param("~status_topic", "/meter_status")
        self.min_repeat_interval = float(rospy.get_param("~min_repeat_interval", 1.0))

        self.is_playing = False
        self.last_msg = None
        self.last_play_time = 0.0

        pygame.mixer.init()
        rospy.Subscriber(self.status_topic, String, self._callback, queue_size=10)
        rospy.loginfo("meter_audio_node ready: topic=%s audio_dir=%s",
                      self.status_topic, self.audio_dir)

    def _find_audio_file(self, base_name):
        for ext in (".wav", ".mp3"):
            path = os.path.join(self.audio_dir, base_name + ext)
            if os.path.exists(path):
                return path
        return None

    def _play_sequence(self, paths):
        self.is_playing = True
        try:
            for path in paths:
                if not path:
                    rospy.logwarn("missing audio file in sequence")
                    continue
                try:
                    pygame.mixer.music.load(path)
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy() and not rospy.is_shutdown():
                        pygame.time.Clock().tick(10)
                except Exception as exc:
                    rospy.logerr("failed to play %s: %s", path, exc)
        finally:
            self.is_playing = False

    def _callback(self, msg):
        data = msg.data.strip()
        now = time.time()
        if self.is_playing:
            return
        if data == self.last_msg and now - self.last_play_time < self.min_repeat_interval:
            return

        parts = [item.strip() for item in data.split(",")]
        if len(parts) == 2:
            region, status = parts
        elif len(parts) == 3:
            _, region, status = parts
        else:
            rospy.logwarn("invalid meter status: %s", data)
            return

        region = region.lower()
        status = status.lower()
        audio_sequence = [
            self._find_audio_file("dashboard_%s" % region),
            self._find_audio_file("display_%s" % status),
        ]
        if status in ("low", "high"):
            audio_sequence.append(self._find_audio_file("status_abnormal"))
        elif status == "normal":
            audio_sequence.append(self._find_audio_file("status_normal"))

        rospy.loginfo("meter status: region=%s status=%s", region.upper(), status)
        self.last_msg = data
        self.last_play_time = now
        self._play_sequence(audio_sequence)


if __name__ == "__main__":
    try:
        MeterAudioNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
