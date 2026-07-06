#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import actionlib
import rospy
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from task_budget import TaskBudget

try:
    import dynamic_reconfigure.client as dynamic_reconfigure_client
except ImportError:
    dynamic_reconfigure_client = None


class SequentialNavInspect:
    def __init__(self):
        rospy.init_node("sequential_nav_inspect", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout", 40.0))
        self.task_budget_reserve_sec = float(rospy.get_param("~task_budget_reserve", 5.0))
        self.min_inspection_point_remaining_sec = float(
            rospy.get_param("~min_inspection_point_remaining", 12.0)
        )
        self.budget = TaskBudget.from_params(default_enabled=False, default_total_sec=300.0)
        self.move_base_wait_timeout_sec = float(rospy.get_param("~move_base_wait_timeout", 90.0))
        self.move_base_retry_interval_sec = float(rospy.get_param("~move_base_retry_interval", 2.0))
        self.detect_timeout_sec = float(rospy.get_param("~detect_timeout", 20.0))
        self.prepare_motion_host_enabled = self._param_bool("~prepare_motion_host", True)
        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.motion_cmd_wait_timeout_sec = float(
            rospy.get_param("~motion_cmd_wait_timeout", 5.0)
        )
        self.motion_prepare_command = rospy.get_param("~motion_prepare_command", "prepare_navigation")
        self.motion_prepare_wait_sec = float(rospy.get_param("~motion_prepare_wait", 1.0))
        self.pre_detect_motion_command = rospy.get_param("~pre_detect_motion_command", "inspection_view_pose")
        self.post_detect_motion_command = rospy.get_param("~post_detect_motion_command", "navigation_view_pose")
        self.detect_motion_wait_sec = float(rospy.get_param("~detect_motion_wait", 0.5))
        self.detect_pose_settle_sec = float(rospy.get_param("~detect_pose_settle", 1.0))
        self.post_detect_hold_sec = float(rospy.get_param("~post_detect_hold", 4.0))

        self.detect_trigger_topic = rospy.get_param("~detect_trigger_topic", "/meter_inspect_trigger")
        self.detect_result_topic = rospy.get_param("~detect_result_topic", "/meter_status")
        self.detect_ready_topic = rospy.get_param("~detect_ready_topic", "/meter_inspection_ready")
        self.detect_ready_timeout_sec = float(rospy.get_param("~detect_ready_timeout", 120.0))
        self.detect_start_command = rospy.get_param("~detect_start_command", "{goal}")
        self.detect_trigger_wait_timeout_sec = float(
            rospy.get_param("~detect_trigger_wait_timeout", 5.0)
        )
        self.report_topic = rospy.get_param("~report_topic", "/inspect_report")
        self.manage_color_stream = self._param_bool("~manage_color_stream", False)
        self.color_stream_required = self._param_bool("~color_stream_required", True)
        self.color_disable_after_detection = self._param_bool(
            "~color_disable_after_detection", True
        )
        self.color_dynamic_reconfigure_node = rospy.get_param(
            "~color_dynamic_reconfigure_node", "/camera/realsense2_camera"
        )
        self.color_enable_param = rospy.get_param("~color_enable_param", "enable_color")
        self.color_image_topic = rospy.get_param("~color_image_topic", "/camera/color/image_raw")
        self.color_stream_wait_timeout = float(rospy.get_param("~color_stream_wait_timeout", 8.0))
        self._color_client = None

        self.goals_yaml = rospy.get_param("~goals_yaml", "")
        if not self.goals_yaml:
            raise rospy.ROSInitException("missing ~goals_yaml")
        if not os.path.isabs(self.goals_yaml):
            self.goals_yaml = os.path.abspath(self.goals_yaml)
        if not os.path.exists(self.goals_yaml):
            raise rospy.ROSInitException("goals_yaml does not exist: %s" % self.goals_yaml)

        seq_param = rospy.get_param("~inspection_sequence", "A,B,C,D")
        if isinstance(seq_param, str):
            self.sequence = [item.strip() for item in seq_param.split(",") if item.strip()]
        else:
            self.sequence = [str(item).strip() for item in seq_param if str(item).strip()]
        if not self.sequence:
            raise rospy.ROSInitException("inspection_sequence is empty")

        self.mb_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        if not self._wait_for_move_base_server():
            raise rospy.ROSInitException("/move_base server is not available")

        self.detect_pub = rospy.Publisher(self.detect_trigger_topic, String, queue_size=10)
        self.report_pub = rospy.Publisher(self.report_topic, String, queue_size=10)
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=1)
        rospy.Subscriber(self.detect_result_topic, String, self._on_detect_result, queue_size=10)

        self._last_detect_result = None
        self.goals = self._load_goals(self.goals_yaml, self.sequence)
        rospy.loginfo("sequential_nav_inspect ready: %s", self.sequence)

    def _wait_for_move_base_server(self):
        start = time.time()
        attempt = 0
        while not rospy.is_shutdown():
            elapsed = time.time() - start
            remaining = self.move_base_wait_timeout_sec - elapsed
            remaining = min(remaining, self.budget.cap_timeout(remaining, reserve=self.task_budget_reserve_sec))
            if remaining <= 0.0:
                return False
            attempt += 1
            wait_slice = min(self.move_base_retry_interval_sec, remaining)
            if self.mb_client.wait_for_server(rospy.Duration(wait_slice)):
                rospy.loginfo("/move_base action server ready")
                return True
            rospy.loginfo(
                "waiting for /move_base action server, attempt=%d elapsed=%.1fs",
                attempt,
                elapsed + wait_slice,
            )
        return False

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _load_goals(self, yaml_path, sequence):
        with open(yaml_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        goals_ns = data.get("goals", {})
        sequences_ns = data.get("sequences", {})
        if len(sequence) == 1 and sequence[0] in sequences_ns:
            sequence = [str(item).strip() for item in sequences_ns[sequence[0]] if str(item).strip()]

        goals = []
        for key in sequence:
            goal_data = goals_ns.get(key)
            if not goal_data:
                rospy.logwarn("goal %s not found in %s", key, yaml_path)
                continue
            pos = goal_data.get("position")
            ori = goal_data.get("orientation")
            if not (
                isinstance(pos, (list, tuple))
                and len(pos) == 3
                and isinstance(ori, (list, tuple))
                and len(ori) == 4
            ):
                rospy.logwarn("goal %s has invalid pose", key)
                continue
            goals.append({"name": key, "position": pos, "orientation": ori})

        if not goals:
            raise rospy.ROSInitException("no valid goals loaded")
        return goals

    def _on_detect_result(self, msg):
        text = str(msg.data).strip()
        if text and text != self.detect_start_command:
            self._last_detect_result = text

    def _prepare_motion_host(self):
        if not self.prepare_motion_host_enabled:
            return True

        rospy.loginfo("preparing Lite3 motion host before sequential navigation")
        return self._publish_motion_command(self.motion_prepare_command, "startup", self.motion_prepare_wait_sec)

    def _publish_motion_command(self, command, label, wait_after):
        if not command:
            return True
        start = time.time()
        timeout = self.budget.cap_timeout(
            self.motion_cmd_wait_timeout_sec,
            reserve=self.task_budget_reserve_sec,
        )
        if timeout <= 0.0:
            return False
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.motion_cmd_pub.get_num_connections() == 0:
            if time.time() - start > timeout:
                rospy.logerr("no subscriber on %s for %s", self.motion_cmd_topic, label)
                return False
            rate.sleep()

        rospy.loginfo("%s motion command: %s", label, command)
        self.motion_cmd_pub.publish(String(data=command))
        self.budget.sleep(wait_after, reserve=self.task_budget_reserve_sec)
        return True

    def _send_goal_and_wait(self, goal_data):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position.x = float(goal_data["position"][0])
        goal.target_pose.pose.position.y = float(goal_data["position"][1])
        goal.target_pose.pose.position.z = float(goal_data["position"][2])
        goal.target_pose.pose.orientation.x = float(goal_data["orientation"][0])
        goal.target_pose.pose.orientation.y = float(goal_data["orientation"][1])
        goal.target_pose.pose.orientation.z = float(goal_data["orientation"][2])
        goal.target_pose.pose.orientation.w = float(goal_data["orientation"][3])

        self.mb_client.send_goal(goal)
        timeout = self.budget.cap_timeout(self.nav_timeout_sec, reserve=self.task_budget_reserve_sec)
        if timeout <= 0.0 or not self.mb_client.wait_for_result(rospy.Duration(timeout)):
            self.mb_client.cancel_goal()
            return False
        return self.mb_client.get_state() == 3

    def _get_color_client(self):
        if self._color_client is not None:
            return self._color_client
        if dynamic_reconfigure_client is None:
            raise RuntimeError("dynamic_reconfigure Python client is not available")
        timeout = self.budget.cap_timeout(3.0, reserve=self.task_budget_reserve_sec)
        self._color_client = dynamic_reconfigure_client.Client(
            self.color_dynamic_reconfigure_node,
            timeout=max(0.1, timeout),
        )
        return self._color_client

    def _set_color_stream(self, enabled):
        if not self.manage_color_stream:
            return True
        try:
            client = self._get_color_client()
            client.update_configuration({self.color_enable_param: bool(enabled)})
            rospy.loginfo("set color stream %s via %s/%s",
                          enabled, self.color_dynamic_reconfigure_node, self.color_enable_param)
            return True
        except Exception as exc:
            rospy.logerr("failed to set color stream %s: %s", enabled, exc)
            return not self.color_stream_required

    def _wait_for_color_frame(self):
        if not self.manage_color_stream:
            return True
        try:
            timeout = self.budget.cap_timeout(self.color_stream_wait_timeout, reserve=self.task_budget_reserve_sec)
            rospy.wait_for_message(
                self.color_image_topic,
                Image,
                timeout=timeout,
            )
            rospy.loginfo("color stream ready: %s", self.color_image_topic)
            return True
        except rospy.ROSException:
            rospy.logerr("no color frame on %s after %.1fs",
                         self.color_image_topic, self.color_stream_wait_timeout)
            return not self.color_stream_required

    def _prepare_color_stream_for_detection(self):
        if not self._set_color_stream(True):
            return False
        return self._wait_for_color_frame()

    def _finish_color_stream_after_detection(self):
        if self.manage_color_stream and self.color_disable_after_detection:
            self._set_color_stream(False)

    def _parse_detection_result(self, text):
        parts = [item.strip() for item in str(text).split(",")]
        if len(parts) == 3:
            return parts[0], parts[1], parts[2]
        if len(parts) == 2:
            return None, parts[0], parts[1]
        return None, None, None

    def _wait_for_detect_ready(self):
        if not self.detect_ready_topic:
            return True, ""
        timeout = self.budget.cap_timeout(self.detect_ready_timeout_sec, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                ready = rospy.wait_for_message(self.detect_ready_topic, Bool, timeout=1.0)
                if ready.data:
                    return True, ""
                self.budget.sleep(0.2, reserve=self.task_budget_reserve_sec)
            except rospy.ROSException:
                pass
        return False, "meter inspection ready timeout"

    def _trigger_and_wait_result(self, goal_name):
        self._last_detect_result = None
        ready_ok, ready_reason = self._wait_for_detect_ready()
        if not ready_ok:
            return False, ready_reason

        start_wait = time.time()
        trigger_timeout = self.budget.cap_timeout(
            self.detect_trigger_wait_timeout_sec,
            reserve=self.task_budget_reserve_sec,
        )
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.detect_pub.get_num_connections() == 0:
            if trigger_timeout <= 0.0 or time.time() - start_wait > trigger_timeout:
                return False, "no subscriber on %s" % self.detect_trigger_topic
            rate.sleep()

        command = self.detect_start_command
        if "{goal}" in command:
            command = command.format(goal=goal_name)
        self.detect_pub.publish(String(data=command))

        start = time.time()
        detect_timeout = self.budget.cap_timeout(self.detect_timeout_sec, reserve=self.task_budget_reserve_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._last_detect_result:
                trigger, region, status = self._parse_detection_result(self._last_detect_result)
                if trigger and trigger != goal_name:
                    rospy.logwarn(
                        "ignore meter result for %s while waiting for %s: %s",
                        trigger,
                        goal_name,
                        self._last_detect_result,
                    )
                    self._last_detect_result = None
                elif region and status:
                    return True, self._last_detect_result
                else:
                    rospy.logwarn("ignore malformed meter result: %s", self._last_detect_result)
                    self._last_detect_result = None
            if detect_timeout <= 0.0 or time.time() - start > detect_timeout:
                return False, "timeout"
            rate.sleep()
        return False, "shutdown"

    def run(self):
        if not self._prepare_motion_host():
            return

        for index, goal_data in enumerate(self.goals, start=1):
            if rospy.is_shutdown():
                break

            name = goal_data["name"]
            if not self.budget.check(name, self.min_inspection_point_remaining_sec):
                break
            rospy.loginfo("[%d/%d] navigating to inspection goal %s",
                          index, len(self.goals), name)
            nav_ok = self._send_goal_and_wait(goal_data)
            if not nav_ok:
                report = "%s navigation failed or timed out" % name
                rospy.logwarn(report)
                self.report_pub.publish(String(data=report))
                continue

            if not self._publish_motion_command(self.pre_detect_motion_command, "pre-detect", self.detect_motion_wait_sec):
                report = "%s inspection skipped: pre-detect motion command failed" % name
                rospy.logwarn(report)
                self.report_pub.publish(String(data=report))
                continue
            if self.detect_pose_settle_sec > 0.0:
                self.budget.sleep(self.detect_pose_settle_sec, reserve=self.task_budget_reserve_sec)

            try:
                if not self._prepare_color_stream_for_detection():
                    report = "%s inspection skipped: color stream unavailable" % name
                    rospy.logwarn(report)
                else:
                    detect_ok, result = self._trigger_and_wait_result(name)
                    if detect_ok:
                        report = "%s inspection result: %s" % (name, result)
                    else:
                        report = "%s inspection timeout" % name
                    if self.post_detect_hold_sec > 0.0:
                        self.budget.sleep(self.post_detect_hold_sec, reserve=self.task_budget_reserve_sec)
            finally:
                self._publish_motion_command(self.post_detect_motion_command, "post-detect", self.detect_motion_wait_sec)
                self._finish_color_stream_after_detection()
            rospy.loginfo(report)
            self.report_pub.publish(String(data=report))

        rospy.loginfo("sequential navigation inspection finished")


if __name__ == "__main__":
    try:
        SequentialNavInspect().run()
    except rospy.ROSInitException as exc:
        rospy.logerr(str(exc))
    except Exception as exc:
        rospy.logerr("sequential_nav_inspect failed: %s", exc)
