#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shlex
import shutil
import subprocess
import threading
import time

import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class MeterDockerInspectionNode:
    def __init__(self):
        rospy.init_node("meter_docker_inspection_node", anonymous=True)

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/meter_inspect_trigger")
        self.result_topic = rospy.get_param("~result_topic", "/meter_status")

        default_host_workspace = rospkg.RosPack().get_path("dog_motion")
        self.host_workspace = rospy.get_param("~host_workspace", default_host_workspace)
        self.container_workspace = rospy.get_param("~container_workspace", "/workspace")
        self.docker_image = rospy.get_param("~docker_image", "yolo11")
        self.docker_command = rospy.get_param("~docker_command", "docker")
        self.model_in_container = rospy.get_param(
            "~model_in_container", "/workspace/models/yuyin.engine"
        )
        self.helper_script_name = rospy.get_param("~helper_script_name", "meter_batch_infer.py")
        self.helper_host_subdir = rospy.get_param("~helper_host_subdir", "docker_tools")
        self.sample_host_subdir = rospy.get_param("~sample_host_subdir", "runtime/meter_samples")

        self.warmup_frames = int(rospy.get_param("~warmup_frames", 15))
        self.sample_count = int(rospy.get_param("~sample_count", 5))
        self.sample_interval = float(rospy.get_param("~sample_interval", 0.15))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.25))
        self.encoding = rospy.get_param("~encoding", "bgr8")

        self.bridge = CvBridge()
        self.lock = threading.Condition()
        self.latest_image = None
        self.frame_seq = 0
        self.running = False

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10, latch=True)
        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(self.trigger_topic, String, self._trigger_callback, queue_size=5)

        self._ensure_helper_script()
        rospy.loginfo(
            "meter_docker_inspection_node ready: trigger=%s image=%s docker=%s",
            self.trigger_topic,
            self.image_topic,
            self.docker_image,
        )

    def _ensure_helper_script(self):
        src = os.path.join(
            rospkg.RosPack().get_path("dog_motion"), "scripts", self.helper_script_name
        )
        dst_dir = os.path.join(self.host_workspace, self.helper_host_subdir)
        dst = os.path.join(dst_dir, self.helper_script_name)
        if not os.path.exists(dst_dir):
            os.makedirs(dst_dir)
        shutil.copyfile(src, dst)
        os.chmod(dst, 0o755)

    def _image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, self.encoding)
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "cv_bridge failed: %s", exc)
            return
        with self.lock:
            self.latest_image = image
            self.frame_seq += 1
            self.lock.notify_all()

    def _wait_next_frame(self, last_seq, timeout):
        deadline = time.time() + timeout
        with self.lock:
            while self.frame_seq <= last_seq and not rospy.is_shutdown():
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None, last_seq
                self.lock.wait(remaining)
            return self.latest_image.copy(), self.frame_seq

    def _trigger_callback(self, msg):
        if self.running:
            rospy.logwarn("docker meter inspection already running, ignored trigger: %s", msg.data)
            return
        threading.Thread(target=self._inspect, args=(msg.data,), daemon=True).start()

    def _inspect(self, trigger_id):
        self.running = True
        try:
            safe_id = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(trigger_id))
            stamp = time.strftime("%Y%m%d_%H%M%S")
            rel_dir = os.path.join(self.sample_host_subdir, "%s_%s" % (stamp, safe_id or "trigger"))
            host_dir = os.path.join(self.host_workspace, rel_dir)
            container_dir = os.path.join(self.container_workspace, rel_dir).replace("\\", "/")
            os.makedirs(host_dir, exist_ok=True)

            rospy.loginfo("meter docker inspection triggered: %s", trigger_id)
            last_seq = self.frame_seq
            for _ in range(max(0, self.warmup_frames)):
                image, last_seq = self._wait_next_frame(last_seq, 2.0)
                if image is None:
                    rospy.logwarn("meter docker inspection timed out during warmup")
                    return

            for index in range(self.sample_count):
                image, last_seq = self._wait_next_frame(last_seq, 2.0)
                if image is None:
                    rospy.logwarn("meter docker inspection timed out while sampling")
                    return
                path = os.path.join(host_dir, "sample_%02d.jpg" % (index + 1))
                cv2.imwrite(path, image)
                if self.sample_interval > 0:
                    rospy.sleep(self.sample_interval)

            result = self._run_docker_infer(container_dir)
            if result:
                result_with_trigger = "%s,%s" % (safe_id, result)
                self.result_pub.publish(String(data=result_with_trigger))
                rospy.loginfo("meter docker inspection final result: %s", result_with_trigger)
        finally:
            self.running = False

    def _run_docker_infer(self, container_image_dir):
        script_in_container = os.path.join(
            self.container_workspace, self.helper_host_subdir, self.helper_script_name
        ).replace("\\", "/")
        shell_cmd = (
            "python3 {script} --model {model} --images {images} --min-confidence {conf}"
        ).format(
            script=shlex.quote(script_in_container),
            model=shlex.quote(self.model_in_container),
            images=shlex.quote(container_image_dir),
            conf=self.min_confidence,
        )
        cmd = (
            shlex.split(self.docker_command)
            + [
                "run",
                "--rm",
                "--runtime=nvidia",
                "--privileged",
                "--network",
                "host",
                "-v",
                "%s:%s" % (self.host_workspace, self.container_workspace),
                "-v",
                "/dev:/dev",
                "-w",
                self.container_workspace,
                "--entrypoint",
                "/bin/bash",
                self.docker_image,
                "-lc",
                shell_cmd,
            ]
        )
        rospy.loginfo("running docker meter inference")
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=120
        )
        if proc.stdout:
            rospy.loginfo("docker stdout:\n%s", proc.stdout.strip())
        if proc.stderr:
            rospy.logwarn("docker stderr:\n%s", proc.stderr.strip())
        if proc.returncode != 0:
            rospy.logerr("docker inference failed: returncode=%s", proc.returncode)
            return None
        for line in proc.stdout.splitlines():
            line = line.strip()
            if line.startswith("RESULT:"):
                return line.split(":", 1)[1].strip()
        rospy.logerr("docker inference completed without RESULT line")
        return None


if __name__ == "__main__":
    try:
        MeterDockerInspectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
