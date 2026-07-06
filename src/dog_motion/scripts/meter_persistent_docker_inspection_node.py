#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import queue
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
from std_msgs.msg import Bool
from std_msgs.msg import String


class MeterPersistentDockerInspectionNode:
    def __init__(self):
        rospy.init_node("meter_persistent_docker_inspection_node", anonymous=True)

        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/meter_inspect_trigger")
        self.result_topic = rospy.get_param("~result_topic", "/meter_status")
        self.ready_topic = rospy.get_param("~ready_topic", "/meter_inspection_ready")

        default_host_workspace = rospkg.RosPack().get_path("dog_motion")
        self.host_workspace = rospy.get_param("~host_workspace", default_host_workspace)
        self.container_workspace = rospy.get_param("~container_workspace", "/workspace")
        self.docker_image = rospy.get_param("~docker_image", "yolo11")
        self.docker_command = rospy.get_param("~docker_command", "docker")
        self.model_in_container = rospy.get_param(
            "~model_in_container", "/workspace/models/yuyin.engine"
        )
        self.helper_script_name = rospy.get_param("~helper_script_name", "meter_persistent_infer.py")
        self.helper_host_subdir = rospy.get_param("~helper_host_subdir", "docker_tools")
        self.sample_host_subdir = rospy.get_param("~sample_host_subdir", "runtime/meter_samples")

        self.warmup_frames = int(rospy.get_param("~warmup_frames", 15))
        self.sample_count = int(rospy.get_param("~sample_count", 5))
        self.sample_interval = float(rospy.get_param("~sample_interval", 0.15))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.25))
        self.encoding = rospy.get_param("~encoding", "bgr8")
        self.container_ready_timeout = float(rospy.get_param("~container_ready_timeout", 90.0))
        self.infer_timeout = float(rospy.get_param("~infer_timeout", 45.0))

        self.bridge = CvBridge()
        self.lock = threading.Condition()
        self.latest_image = None
        self.frame_seq = 0
        self.running = False
        self.container_ready = False
        self.proc = None
        self.result_queue = queue.Queue()

        self.result_pub = rospy.Publisher(self.result_topic, String, queue_size=10, latch=True)
        self.ready_pub = rospy.Publisher(self.ready_topic, Bool, queue_size=1, latch=True)
        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(self.trigger_topic, String, self._trigger_callback, queue_size=5)

        self.ready_pub.publish(Bool(data=False))
        self._ensure_helper_script()
        self._start_container()
        self.container_ready = True
        self.ready_pub.publish(Bool(data=True))
        rospy.on_shutdown(self._shutdown_container)
        rospy.loginfo(
            "meter_persistent_docker_inspection_node ready: trigger=%s image=%s docker=%s",
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

    def _start_container(self):
        script_in_container = os.path.join(
            self.container_workspace, self.helper_host_subdir, self.helper_script_name
        ).replace("\\", "/")
        shell_cmd = (
            "python3 -u {script} --model {model} --min-confidence {conf}"
        ).format(
            script=shlex.quote(script_in_container),
            model=shlex.quote(self.model_in_container),
            conf=self.min_confidence,
        )
        cmd = (
            shlex.split(self.docker_command)
            + [
                "run",
                "--rm",
                "-i",
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
        rospy.loginfo("starting persistent docker meter inference")
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._read_container_output, daemon=True).start()

        deadline = time.time() + self.container_ready_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                kind, payload = self.result_queue.get(timeout=0.2)
            except queue.Empty:
                if self.proc.poll() is not None:
                    raise RuntimeError("persistent docker exited before READY")
                continue
            if kind == "ready":
                rospy.loginfo("persistent docker meter inference is ready")
                return
            rospy.loginfo("docker startup: %s", payload)
        raise RuntimeError("persistent docker did not become ready within %.1fs" % self.container_ready_timeout)

    def _read_container_output(self):
        for raw_line in self.proc.stdout:
            line = raw_line.strip()
            if not line:
                continue
            if line == "READY":
                self.result_queue.put(("ready", line))
            elif line.startswith("RESULT:"):
                self.result_queue.put(("result", line.split(":", 1)[1].strip()))
            elif line.startswith("ERROR:"):
                self.result_queue.put(("error", line.split(":", 1)[1].strip()))
            else:
                rospy.loginfo("docker: %s", line)

    def _shutdown_container(self):
        self.container_ready = False
        self.ready_pub.publish(Bool(data=False))
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None and self.proc.stdin:
                self.proc.stdin.write("QUIT\n")
                self.proc.stdin.flush()
                self.proc.wait(timeout=3.0)
        except Exception:
            if self.proc.poll() is None:
                self.proc.terminate()

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
        if not self.container_ready:
            rospy.logwarn("persistent meter inspection is not ready yet, ignored trigger: %s", msg.data)
            return
        if self.running:
            rospy.logwarn("persistent meter inspection already running, ignored trigger: %s", msg.data)
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

            rospy.loginfo("persistent meter inspection triggered: %s", trigger_id)
            last_seq = self.frame_seq
            for _ in range(max(0, self.warmup_frames)):
                image, last_seq = self._wait_next_frame(last_seq, 2.0)
                if image is None:
                    rospy.logwarn("persistent meter inspection timed out during warmup")
                    return

            for index in range(self.sample_count):
                image, last_seq = self._wait_next_frame(last_seq, 2.0)
                if image is None:
                    rospy.logwarn("persistent meter inspection timed out while sampling")
                    return
                path = os.path.join(host_dir, "sample_%02d.jpg" % (index + 1))
                cv2.imwrite(path, image)
                if self.sample_interval > 0:
                    rospy.sleep(self.sample_interval)

            result = self._run_persistent_infer(safe_id, container_dir)
            if result:
                self.result_pub.publish(String(data=result))
                rospy.loginfo("persistent meter inspection final result: %s", result)
        finally:
            self.running = False

    def _run_persistent_infer(self, trigger_id, container_image_dir):
        if self.proc is None or self.proc.poll() is not None:
            rospy.logerr("persistent docker is not running")
            return None
        job = {
            "trigger": trigger_id,
            "images": container_image_dir,
            "min_confidence": self.min_confidence,
        }
        self.proc.stdin.write(json.dumps(job, sort_keys=True) + "\n")
        self.proc.stdin.flush()

        deadline = time.time() + self.infer_timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                kind, payload = self.result_queue.get(timeout=0.2)
            except queue.Empty:
                if self.proc.poll() is not None:
                    rospy.logerr("persistent docker exited during inference")
                    return None
                continue
            if kind == "result":
                parts = [part.strip() for part in payload.split(",")]
                if len(parts) == 3 and parts[0] == trigger_id:
                    return payload
                rospy.logwarn("ignored stale persistent docker result: %s", payload)
            elif kind == "error":
                rospy.logerr("persistent docker inference error: %s", payload)
                return None
        rospy.logerr("persistent docker inference timed out after %.1fs", self.infer_timeout)
        return None


if __name__ == "__main__":
    try:
        MeterPersistentDockerInspectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("meter_persistent_docker_inspection_node failed: %s", exc)
