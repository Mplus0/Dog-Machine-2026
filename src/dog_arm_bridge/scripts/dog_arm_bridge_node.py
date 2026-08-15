#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class DogArmBridgeNode(object):
    def __init__(self):
        rospy.init_node("dog_arm_bridge_node", anonymous=False)

        self.task_cmd_topic = rospy.get_param("~task_cmd_topic", "/dog_arm/task_cmd")
        self.task_result_topic = rospy.get_param("~task_result_topic", "/dog_arm/task_result")
        self.base_adjust_req_topic = rospy.get_param("~base_adjust_req_topic", "/dog_arm/base_adjust_req")
        self.local_task_cmd_topic = rospy.get_param("~local_task_cmd_topic", "/dog_arm/local_task_cmd")
        self.task_result_event_topic = rospy.get_param("~task_result_event_topic", "/dog_arm/task_result_event")
        self.base_adjust_event_topic = rospy.get_param("~base_adjust_event_topic", "/dog_arm/base_adjust_event")

        self.enable_base_adjust_execution = self._param_bool("~enable_base_adjust_execution", False)
        self.base_adjust_mode = str(rospy.get_param("~base_adjust_mode", "cmd_vel")).strip().lower()
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.base_adjust_speed_mps = abs(float(rospy.get_param("~base_adjust_speed_mps", 0.05)))
        self.base_adjust_max_step_m = abs(float(rospy.get_param("~base_adjust_max_step_m", 0.20)))
        self.base_adjust_stop_sec = max(0.0, float(rospy.get_param("~base_adjust_stop_sec", 0.20)))

        self.task_prefix = rospy.get_param("~task_id_prefix", "dog")
        self.task_seq = int(rospy.get_param("~task_id_start", 1))

        self.task_cmd_pub = rospy.Publisher(self.task_cmd_topic, String, queue_size=10)
        self.task_result_event_pub = rospy.Publisher(self.task_result_event_topic, String, queue_size=10)
        self.base_adjust_event_pub = rospy.Publisher(self.base_adjust_event_topic, String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=10)

        self.local_task_sub = rospy.Subscriber(self.local_task_cmd_topic, String, self._on_local_task_cmd, queue_size=10)
        self.task_result_sub = rospy.Subscriber(self.task_result_topic, String, self._on_task_result, queue_size=10)
        self.base_adjust_sub = rospy.Subscriber(
            self.base_adjust_req_topic, String, self._on_base_adjust_req, queue_size=10
        )

        rospy.loginfo("dog_arm_bridge ready")
        rospy.loginfo("publishing arm task commands on %s", self.task_cmd_topic)
        rospy.loginfo("listening arm results on %s", self.task_result_topic)
        rospy.loginfo("listening base adjust requests on %s", self.base_adjust_req_topic)
        rospy.loginfo("base adjust execution enabled=%s mode=%s", self.enable_base_adjust_execution, self.base_adjust_mode)

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _next_task_id(self):
        task_id = "%s_%04d" % (self.task_prefix, self.task_seq)
        self.task_seq += 1
        return task_id

    def _decode_json(self, text, label):
        try:
            value = json.loads(text)
        except ValueError as exc:
            rospy.logwarn("invalid %s JSON: %s; raw=%s", label, exc, text)
            return None
        if not isinstance(value, dict):
            rospy.logwarn("invalid %s JSON: expected object; raw=%s", label, text)
            return None
        return value

    def _encode_json(self, data):
        return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":"))

    def _normalize_local_task_cmd(self, raw_text):
        text = raw_text.strip()
        if not text:
            rospy.logwarn("empty local arm task command ignored")
            return None

        if text.startswith("{"):
            data = self._decode_json(text, "local task command")
            if data is None:
                return None
        else:
            parts = [part for part in text.replace(":", " ").replace(",", " ").split() if part]
            cmd = parts[0]
            data = {
                "task_id": self._next_task_id(),
                "cmd": cmd,
            }
            if len(parts) > 1:
                data["zone"] = parts[1]

        data["task_id"] = str(data.get("task_id") or self._next_task_id())
        data["cmd"] = str(data.get("cmd", "")).strip()
        if not data["cmd"]:
            rospy.logwarn("local arm task command missing cmd: %s", raw_text)
            return None

        aliases = {
            "place": "place_to_zone",
            "place_zone": "place_to_zone",
        }
        data["cmd"] = aliases.get(data["cmd"], data["cmd"])
        return data

    def _on_local_task_cmd(self, msg):
        data = self._normalize_local_task_cmd(msg.data)
        if data is None:
            return
        payload = self._encode_json(data)
        rospy.loginfo("dog -> arm task_cmd: %s", payload)
        self.task_cmd_pub.publish(String(data=payload))

    def _on_task_result(self, msg):
        data = self._decode_json(msg.data, "arm task result")
        if data is None:
            return
        if "task_id" not in data or "result" not in data:
            rospy.logwarn("arm task result missing task_id/result: %s", msg.data)
            return
        payload = self._encode_json(data)
        rospy.loginfo("arm -> dog task_result: %s", payload)
        self.task_result_event_pub.publish(String(data=payload))

    def _on_base_adjust_req(self, msg):
        data = self._decode_json(msg.data, "base adjust request")
        if data is None:
            return

        direction = str(data.get("direction", "")).strip().lower()
        try:
            step_m = abs(float(data.get("step_m", 0.0)))
        except (TypeError, ValueError):
            rospy.logwarn("base adjust request has invalid step_m: %s", msg.data)
            return

        if direction not in ("left", "right"):
            rospy.logwarn("unsupported base adjust direction: %s", msg.data)
            return
        if step_m <= 0.0:
            rospy.logwarn("base adjust step_m must be positive: %s", msg.data)
            return
        if step_m > self.base_adjust_max_step_m:
            rospy.logwarn("base adjust step %.3fm capped to %.3fm", step_m, self.base_adjust_max_step_m)
            step_m = self.base_adjust_max_step_m
            data["step_m"] = step_m

        if not self.enable_base_adjust_execution:
            rospy.loginfo("base adjust execution disabled; request recorded only")
            data["executed"] = False
            data["status"] = "execution_disabled"
            self._publish_base_adjust_event(data)
            return
        executed = False
        if self.base_adjust_mode == "cmd_vel":
            executed = self._execute_cmd_vel_adjust(direction, step_m)
        elif self.base_adjust_mode == "lite3_motion_cmd":
            executed = self._execute_lite3_motion_cmd_adjust(direction, step_m)
        else:
            rospy.logwarn("unknown base_adjust_mode=%s; request not executed", self.base_adjust_mode)
        data["executed"] = bool(executed)
        data["status"] = "completed" if executed else "execution_failed"
        self._publish_base_adjust_event(data)

    def _publish_base_adjust_event(self, data):
        payload = self._encode_json(data)
        rospy.loginfo("arm -> dog base_adjust_event: %s", payload)
        self.base_adjust_event_pub.publish(String(data=payload))

    def _execute_cmd_vel_adjust(self, direction, step_m):
        if self.base_adjust_speed_mps <= 0.0:
            rospy.logwarn("base_adjust_speed_mps must be positive")
            return False
        duration = step_m / self.base_adjust_speed_mps
        twist = Twist()
        if direction == "left":
            twist.linear.y = abs(self.base_adjust_speed_mps)
        else:
            twist.linear.y = -abs(self.base_adjust_speed_mps)

        rospy.logwarn("executing base adjust via cmd_vel: direction=%s step=%.3fm duration=%.2fs", direction, step_m, duration)
        rate = rospy.Rate(20)
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self._publish_zero_cmd_vel(self.base_adjust_stop_sec)
        return True

    def _execute_lite3_motion_cmd_adjust(self, direction, step_m):
        command = "side 10000" if direction == "left" else "side -10000"
        rospy.logwarn("publishing experimental lite3 motion command for base adjust: %s", command)
        self.motion_cmd_pub.publish(String(data=command))
        return True

    def _publish_zero_cmd_vel(self, duration):
        zero = Twist()
        if duration <= 0.0:
            self.cmd_vel_pub.publish(zero)
            return
        rate = rospy.Rate(20)
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.cmd_vel_pub.publish(zero)
            rate.sleep()
        self.cmd_vel_pub.publish(zero)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        DogArmBridgeNode().spin()
    except rospy.ROSInterruptException:
        pass
