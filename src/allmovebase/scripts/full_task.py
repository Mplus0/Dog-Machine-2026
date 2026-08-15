#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

from allmovebase.dog_arm_task_client import DogArmTaskClient
from allmovebase.task_budget import TaskBudget

try:
    import dynamic_reconfigure.client as dynamic_reconfigure_client
except ImportError:
    dynamic_reconfigure_client = None


class FullTask:
    def __init__(self):
        rospy.init_node("full_task", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.goals_yaml = rospy.get_param("~goals_yaml", "")
        self.state_file = rospy.get_param("~meter_state_file", "")
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout", 45.0))
        self.task_budget_start_after_prerequisites = self._param_bool("~task_budget_start_after_prerequisites", True)
        self.task_budget_reserve_sec = float(rospy.get_param("~task_budget_reserve", 5.0))
        self.min_obstacle_remaining_sec = float(rospy.get_param("~min_obstacle_remaining", 30.0))
        self.min_inspection_remaining_sec = float(rospy.get_param("~min_inspection_remaining", 60.0))
        self.min_pick_place_remaining_sec = float(rospy.get_param("~min_pick_place_remaining", 45.0))
        self.budget = TaskBudget.from_params(
            default_enabled=True,
            default_total_sec=300.0,
            start_now=not self.task_budget_start_after_prerequisites,
        )
        self.move_base_wait_timeout_sec = float(rospy.get_param("~move_base_wait_timeout", 120.0))
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.require_map = self._param_bool("~require_map", True)
        self.require_scan = self._param_bool("~require_scan", True)
        self.require_tf = self._param_bool("~require_tf", True)
        self.map_wait_timeout_sec = float(rospy.get_param("~map_wait_timeout", 30.0))
        self.scan_wait_timeout_sec = float(rospy.get_param("~scan_wait_timeout", 10.0))
        self.tf_wait_timeout_sec = float(rospy.get_param("~tf_wait_timeout", 30.0))
        self.require_move_base_status = self._param_bool("~require_move_base_status", True)
        self.move_base_status_topic = rospy.get_param("~move_base_status_topic", "/move_base/status")
        self.move_base_status_wait_timeout_sec = float(rospy.get_param("~move_base_status_wait_timeout", 45.0))

        self.run_obstacle = self._param_bool("~run_obstacle", True)
        self.run_inspection = self._param_bool("~run_inspection", True)
        self.run_pick_place = self._param_bool("~run_pick_place", True)
        self.obstacle_sequence = self._parse_list(rospy.get_param("~obstacle_sequence", "obstacle_test"))
        self.inspection_sequence = self._parse_list(rospy.get_param("~inspection_sequence", "recognition"))
        self.region_order = [item.upper() for item in self._parse_list(rospy.get_param("~region_order", "A,B,C,D"))]
        self.max_abnormal_count = int(rospy.get_param("~max_abnormal_count", 2))

        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.prepare_motion_host_enabled = self._param_bool("~prepare_motion_host", True)
        self.motion_prepare_command = rospy.get_param("~motion_prepare_command", "prepare_navigation")
        self.motion_cmd_wait_timeout_sec = float(rospy.get_param("~motion_cmd_wait_timeout", 5.0))
        self.motion_prepare_wait_sec = float(rospy.get_param("~motion_prepare_wait", 1.0))

        self.detect_trigger_topic = rospy.get_param("~detect_trigger_topic", "/meter_inspect_trigger")
        self.detect_result_topic = rospy.get_param("~detect_result_topic", "/meter_status")
        self.detect_ready_topic = rospy.get_param("~detect_ready_topic", "/meter_inspection_ready")
        self.detect_ready_timeout_sec = float(rospy.get_param("~detect_ready_timeout", 120.0))
        self.detect_timeout_sec = float(rospy.get_param("~detect_timeout", 45.0))
        self.pre_detect_motion_command = rospy.get_param("~pre_detect_motion_command", "inspection_view_pose")
        self.post_detect_motion_command = rospy.get_param("~post_detect_motion_command", "navigation_view_pose")
        self.detect_motion_wait_sec = float(rospy.get_param("~detect_motion_wait", 0.5))
        self.detect_pose_settle_sec = float(rospy.get_param("~detect_pose_settle", 1.0))
        self.post_detect_hold_sec = float(rospy.get_param("~post_detect_hold", 4.0))
        self.manage_color_stream = self._param_bool("~manage_color_stream", True)
        self.color_stream_required = self._param_bool("~color_stream_required", True)
        self.color_disable_after_detection = self._param_bool("~color_disable_after_detection", True)
        self.color_dynamic_reconfigure_node = rospy.get_param("~color_dynamic_reconfigure_node", "/camera/realsense2_camera")
        self.color_enable_param = rospy.get_param("~color_enable_param", "enable_color")
        self.color_image_topic = rospy.get_param("~color_image_topic", "/camera/color/image_raw")
        self.color_stream_wait_timeout = float(rospy.get_param("~color_stream_wait_timeout", 8.0))
        self._color_client = None

        self.arm_cmd_topic = rospy.get_param("~arm_cmd_topic", "/dog_arm/task_cmd")
        self.arm_result_topic = rospy.get_param("~arm_result_topic", "/dog_arm/task_result")
        self.arm_base_adjust_event_topic = rospy.get_param("~arm_base_adjust_event_topic", "/dog_arm/base_adjust_event")
        self.arm_transport_connected_topic = rospy.get_param("~arm_transport_connected_topic", "/dog_arm/transport_connected")
        self.arm_pick_command = rospy.get_param("~arm_pick_command", "pick")
        self.arm_place_command_template = rospy.get_param("~arm_place_command", "place_to_zone")
        self.arm_wait_sec = float(rospy.get_param("~arm_wait", 120.0))
        self.arm_command_required = self._param_bool("~arm_command_required", False)
        self.arm_require_transport_connected = self._param_bool("~arm_require_transport_connected", True)
        self.arm_pick_timeout_sec = float(rospy.get_param("~arm_pick_timeout", 180.0))
        self.arm_place_timeout_sec = float(rospy.get_param("~arm_place_timeout", 60.0))
        self.arm_base_adjust_settle_sec = float(rospy.get_param("~arm_base_adjust_settle", 1.0))
        self.arm_base_adjust_event_timeout_sec = float(rospy.get_param("~arm_base_adjust_event_timeout", 5.0))
        self.arm_max_pick_adjust_retries = int(rospy.get_param("~arm_max_pick_adjust_retries", 1))

        self.report_pub = rospy.Publisher("/full_task/report", String, queue_size=10)
        self.done_pub = rospy.Publisher("/full_task/succeeded", Bool, queue_size=1, latch=True)
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=1)
        self.detect_pub = rospy.Publisher(self.detect_trigger_topic, String, queue_size=10)
        rospy.Subscriber(self.detect_result_topic, String, self._on_detect_result, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.mb_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self._last_detect_result = None
        self.meter_states = {}
        self.poses, self.sequences = self._load_pose_registry()
        self.pickup_key, self.place_keys = self._load_pick_place_keys()
        self.arm_client = DogArmTaskClient(
            task_cmd_topic=self.arm_cmd_topic,
            task_result_topic=self.arm_result_topic,
            base_adjust_event_topic=self.arm_base_adjust_event_topic,
            transport_connected_topic=self.arm_transport_connected_topic,
            wait_for_connection_timeout=self.motion_cmd_wait_timeout_sec,
            require_transport_connected=self.arm_require_transport_connected,
            base_adjust_event_timeout=self.arm_base_adjust_event_timeout_sec,
            base_adjust_settle_sec=self.arm_base_adjust_settle_sec,
            max_pick_adjust_retries=self.arm_max_pick_adjust_retries,
            report=self._report,
        )

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _parse_list(self, value):
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item).strip() for item in value if str(item).strip()]

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def _load_pose_registry(self):
        if not self.goals_yaml:
            raise rospy.ROSInitException("missing ~goals_yaml")
        if not os.path.exists(self.goals_yaml):
            raise rospy.ROSInitException("goals_yaml does not exist: %s" % self.goals_yaml)
        data = self._load_yaml(self.goals_yaml)
        poses = {}
        for namespace in ("goals", "waypoints"):
            values = data.get(namespace, {})
            if isinstance(values, dict):
                poses.update(values)
        return poses, data.get("sequences", {}) or {}

    def _expand_sequence(self, value):
        if len(value) == 1 and value[0] in self.sequences and isinstance(self.sequences[value[0]], list):
            return [str(item).strip() for item in self.sequences[value[0]] if str(item).strip()]
        return value

    def _load_pick_place_keys(self):
        pick_place = self.sequences.get("pick_place", {})
        pickup_key = str(pick_place.get("pickup", "pickup_pose")).strip()
        place_keys = pick_place.get("places", {}) or {}
        return pickup_key, {
            region: str(place_keys.get(region, "place_pose_%s" % region)).strip()
            for region in self.region_order
        }

    def _on_detect_result(self, msg):
        data = str(msg.data).strip()
        if not data:
            return
        self._last_detect_result = data
        trigger, region, status = self._parse_detection_result(data)
        if region and status:
            self.meter_states[region.upper()] = status.lower()

    def _parse_detection_result(self, text):
        parts = [item.strip() for item in str(text).split(",")]
        if len(parts) == 3:
            return parts[0], parts[1], parts[2]
        if len(parts) == 2:
            return None, parts[0], parts[1]
        return None, None, None

    def _report(self, text):
        rospy.loginfo(text)
        self.report_pub.publish(String(data=text))

    def _wait_for_connection(self, publisher, topic, timeout):
        timeout = self.budget.cap_timeout(timeout, reserve=self.task_budget_reserve_sec)
        if timeout <= 0.0:
            return False
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
            if time.time() - start > timeout:
                return False
            rate.sleep()
        return True

    def _prepare_motion_host(self):
        if not self.prepare_motion_host_enabled:
            return True
        if not self._wait_for_connection(self.motion_cmd_pub, self.motion_cmd_topic, self.motion_cmd_wait_timeout_sec):
            self._report("no subscriber on %s" % self.motion_cmd_topic)
            return False
        self._report("Lite3 motion prepare: %s" % self.motion_prepare_command)
        self.motion_cmd_pub.publish(String(data=self.motion_prepare_command))
        self.budget.sleep(self.motion_prepare_wait_sec, reserve=self.task_budget_reserve_sec)
        return True

    def _wait_prerequisites(self):
        move_base_wait = self.budget.cap_timeout(self.move_base_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
        if move_base_wait <= 0.0 or not self.mb_client.wait_for_server(rospy.Duration(move_base_wait)):
            self._report("/move_base server is not available")
            return False
        if self.require_move_base_status:
            try:
                timeout = self.budget.cap_timeout(
                    self.move_base_status_wait_timeout_sec,
                    reserve=self.task_budget_reserve_sec,
                )
                rospy.wait_for_message(
                    self.move_base_status_topic,
                    GoalStatusArray,
                    timeout=timeout,
                )
            except rospy.ROSException:
                self._report("move_base status timeout")
                return False
        if self.require_map:
            try:
                timeout = self.budget.cap_timeout(self.map_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
                rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=timeout)
            except rospy.ROSException:
                self._report("map timeout")
                return False
        if self.require_scan:
            try:
                timeout = self.budget.cap_timeout(self.scan_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
                rospy.wait_for_message(self.scan_topic, LaserScan, timeout=timeout)
            except rospy.ROSException:
                self._report("scan timeout")
                return False
        if self.require_tf:
            try:
                timeout = self.budget.cap_timeout(self.tf_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
                self.tf_listener.waitForTransform(
                    self.frame_id,
                    self.base_frame_id,
                    rospy.Time(0),
                    rospy.Duration(timeout),
                )
            except Exception as exc:
                self._report("TF wait failed: %s" % exc)
                return False
        return True

    def _make_goal(self, pose_key):
        pose_data = self.poses.get(pose_key)
        if not pose_data:
            raise ValueError("pose %s not found in %s" % (pose_key, self.goals_yaml))
        pos = pose_data.get("position")
        ori = pose_data.get("orientation")
        if not (
            isinstance(pos, (list, tuple)) and len(pos) == 3
            and isinstance(ori, (list, tuple)) and len(ori) == 4
        ):
            raise ValueError("pose %s has invalid position/orientation" % pose_key)
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position.x = float(pos[0])
        goal.target_pose.pose.position.y = float(pos[1])
        goal.target_pose.pose.position.z = float(pos[2])
        goal.target_pose.pose.orientation.x = float(ori[0])
        goal.target_pose.pose.orientation.y = float(ori[1])
        goal.target_pose.pose.orientation.z = float(ori[2])
        goal.target_pose.pose.orientation.w = float(ori[3])
        return goal

    def _navigate_to(self, pose_key, label):
        if not self.budget.check("navigate/%s" % label, self.task_budget_reserve_sec):
            return False
        self._report("navigate to %s (%s)" % (label, pose_key))
        self.mb_client.send_goal(self._make_goal(pose_key))
        timeout = self.budget.cap_timeout(self.nav_timeout_sec, reserve=self.task_budget_reserve_sec)
        if timeout <= 0.0 or not self.mb_client.wait_for_result(rospy.Duration(timeout)):
            self.mb_client.cancel_goal()
            self._report("navigation timeout: %s, timeout=%.1fs" % (label, timeout))
            return False
        ok = self.mb_client.get_state() == 3
        if not ok:
            self._report("navigation failed: %s, state=%s" % (label, self.mb_client.get_state()))
        return ok

    def _publish_motion_command(self, command, label, wait_after):
        if not command:
            return True
        if not self._wait_for_connection(self.motion_cmd_pub, self.motion_cmd_topic, self.motion_cmd_wait_timeout_sec):
            self._report("no subscriber on %s for %s" % (self.motion_cmd_topic, label))
            return False
        self._report("%s motion command: %s" % (label, command))
        self.motion_cmd_pub.publish(String(data=command))
        self.budget.sleep(wait_after, reserve=self.task_budget_reserve_sec)
        return True

    def _get_color_client(self):
        if self._color_client is not None:
            return self._color_client
        if dynamic_reconfigure_client is None:
            raise RuntimeError("dynamic_reconfigure Python client is not available")
        timeout = self.budget.cap_timeout(3.0, reserve=self.task_budget_reserve_sec)
        self._color_client = dynamic_reconfigure_client.Client(self.color_dynamic_reconfigure_node, timeout=max(0.1, timeout))
        return self._color_client

    def _set_color_stream(self, enabled):
        if not self.manage_color_stream:
            return True
        try:
            self._get_color_client().update_configuration({self.color_enable_param: bool(enabled)})
            self._report("set color stream %s" % enabled)
            return True
        except Exception as exc:
            self._report("failed to set color stream %s: %s" % (enabled, exc))
            return not self.color_stream_required

    def _wait_for_color_frame(self):
        if not self.manage_color_stream:
            return True
        try:
            timeout = self.budget.cap_timeout(self.color_stream_wait_timeout, reserve=self.task_budget_reserve_sec)
            rospy.wait_for_message(self.color_image_topic, Image, timeout=timeout)
            return True
        except rospy.ROSException:
            self._report("color frame timeout")
            return not self.color_stream_required

    def _wait_detect_ready(self):
        timeout = self.budget.cap_timeout(self.detect_ready_timeout_sec, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                ready = rospy.wait_for_message(self.detect_ready_topic, Bool, timeout=1.0)
                if ready.data:
                    return True
            except rospy.ROSException:
                pass
        return False

    def _inspect_at(self, pose_key):
        if not self.budget.check("inspection/%s" % pose_key, self.task_budget_reserve_sec):
            return False
        if not self._publish_motion_command(self.pre_detect_motion_command, "pre-detect", self.detect_motion_wait_sec):
            return False
        self.budget.sleep(self.detect_pose_settle_sec, reserve=self.task_budget_reserve_sec)
        try:
            if not self._set_color_stream(True) or not self._wait_for_color_frame():
                return False
            if not self._wait_detect_ready():
                self._report("meter inspection is not ready")
                return False
            if not self._wait_for_connection(self.detect_pub, self.detect_trigger_topic, 5.0):
                self._report("no subscriber on %s" % self.detect_trigger_topic)
                return False
            self._last_detect_result = None
            self.detect_pub.publish(String(data=pose_key))
            start = time.time()
            detect_timeout = self.budget.cap_timeout(self.detect_timeout_sec, reserve=self.task_budget_reserve_sec)
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                if self._last_detect_result:
                    trigger, region, status = self._parse_detection_result(self._last_detect_result)
                    if trigger and trigger != pose_key:
                        self._last_detect_result = None
                    elif region and status:
                        self._report("inspection result: %s" % self._last_detect_result)
                        return True
                    else:
                        self._last_detect_result = None
                if time.time() - start > detect_timeout:
                    self._report("inspection timeout: %s" % pose_key)
                    return False
                rate.sleep()
        finally:
            self.budget.sleep(self.post_detect_hold_sec, reserve=self.task_budget_reserve_sec)
            self._publish_motion_command(self.post_detect_motion_command, "post-detect", self.detect_motion_wait_sec)
            if self.manage_color_stream and self.color_disable_after_detection:
                self._set_color_stream(False)

    def _load_meter_states_from_file(self):
        if not self.state_file or not os.path.exists(self.state_file):
            return {}
        data = self._load_yaml(self.state_file)
        states = data.get("states", data)
        if not isinstance(states, dict):
            return {}
        return {str(k).strip().upper(): str(v).strip().lower() for k, v in states.items()}

    def _abnormal_regions(self):
        states = self._load_meter_states_from_file()
        states.update(self.meter_states)
        return [r for r in self.region_order if states.get(r, "") and states.get(r) != "normal"][: self.max_abnormal_count]

    def _publish_arm_command(self, command, label):
        if self.arm_command_required and not self.arm_client.wait_for_server(self.motion_cmd_wait_timeout_sec):
            self._report("no subscriber on %s for %s" % (self.arm_cmd_topic, label))
            return False
        if str(command).strip().split(":", 1)[0] == "pick":
            timeout = self.budget.cap_timeout(self.arm_pick_timeout_sec, reserve=self.task_budget_reserve_sec)
            return self.arm_client.pick_with_adjust_retry(label, timeout)
        timeout = self.budget.cap_timeout(self.arm_place_timeout_sec, reserve=self.task_budget_reserve_sec)
        return self.arm_client.place_to_zone(label, timeout)

    def _run_obstacle(self):
        if not self.budget.check("obstacle stage", self.min_obstacle_remaining_sec):
            return False
        self.budget.log_state("obstacle stage start")
        for pose_key in self._expand_sequence(self.obstacle_sequence):
            if not self._navigate_to(pose_key, "obstacle/%s" % pose_key):
                return False
        return True

    def _run_inspection(self):
        if not self.budget.check("inspection stage", self.min_inspection_remaining_sec):
            return False
        self.budget.log_state("inspection stage start")
        for pose_key in self._expand_sequence(self.inspection_sequence):
            if not self._navigate_to(pose_key, "inspection/%s" % pose_key):
                return False
            if not self._inspect_at(pose_key):
                return False
        return True

    def _run_pick_place(self):
        if not self.budget.check("pick-place stage", self.min_pick_place_remaining_sec):
            return False
        self.budget.log_state("pick-place stage start")
        abnormal = self._abnormal_regions()
        if not abnormal:
            self._report("pick-place skipped: no abnormal meter states found")
            return True
        self._report("pick-place abnormal regions: %s" % ",".join(abnormal))
        for index, region in enumerate(abnormal, start=1):
            if not self._navigate_to(self.pickup_key, "pickup %d" % index):
                return False
            if not self._publish_arm_command(self.arm_pick_command, "pickup %d" % index):
                return False
            place_key = self.place_keys.get(region)
            if not self._navigate_to(place_key, "place %s" % region):
                return False
            command = self.arm_place_command_template.format(region=region, index=index)
            if not self._publish_arm_command(command, "place %s" % region):
                return False
        return True

    def run(self):
        self.done_pub.publish(Bool(data=False))
        if not self._prepare_motion_host() or not self._wait_prerequisites():
            return
        self.budget.start()
        self.budget.log_state("full task timed section start")
        ok = True
        if self.run_obstacle:
            ok = ok and self._run_obstacle()
        if ok and self.run_inspection:
            ok = ok and self._run_inspection()
        if ok and self.run_pick_place:
            ok = ok and self._run_pick_place()
        self.done_pub.publish(Bool(data=ok))
        self._report("full task finished: %s" % ("success" if ok else "failed"))


if __name__ == "__main__":
    try:
        FullTask().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("full_task failed: %s", exc)
