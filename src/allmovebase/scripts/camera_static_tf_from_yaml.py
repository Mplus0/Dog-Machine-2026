#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import tf
import yaml


class CameraStaticTfFromYaml:
    def __init__(self):
        rospy.init_node("camera_static_tf_from_yaml", anonymous=False)
        self.yaml_path = rospy.get_param("~yaml_path", "")
        self.transform_key = rospy.get_param(
            "~transform_key", "camera_to_base_link_transform"
        )
        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))

        if not self.yaml_path:
            raise rospy.ROSInitException("missing ~yaml_path")
        if not os.path.exists(self.yaml_path):
            raise rospy.ROSInitException("yaml_path does not exist: %s" % self.yaml_path)

        self.parent_frame, self.child_frame, self.translation, self.rotation = self._load_transform()
        self.broadcaster = tf.TransformBroadcaster()
        rospy.loginfo(
            "camera_static_tf_from_yaml: %s -> %s, xyz=%s, q=%s",
            self.parent_frame,
            self.child_frame,
            self.translation,
            self.rotation,
        )

    def _load_transform(self):
        with open(self.yaml_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        config = data.get(self.transform_key, data)
        parent = str(config.get("parent_frame", "base_link")).strip().lstrip("/")
        child = str(config.get("child_frame", "camera_link")).strip().lstrip("/")
        translation = config.get("translation", [0.2, 0.0, 0.0])
        rotation = config.get("rotation", [0.0, 0.0, 0.0, 1.0])
        if len(translation) != 3:
            raise rospy.ROSInitException("translation must have 3 elements")
        if len(rotation) != 4:
            raise rospy.ROSInitException("rotation must have 4 elements")
        return (
            parent,
            child,
            tuple(float(v) for v in translation),
            tuple(float(v) for v in rotation),
        )

    def spin(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(
                self.translation,
                self.rotation,
                rospy.Time.now(),
                self.child_frame,
                self.parent_frame,
            )
            rate.sleep()


if __name__ == "__main__":
    try:
        CameraStaticTfFromYaml().spin()
    except rospy.ROSInterruptException:
        pass
