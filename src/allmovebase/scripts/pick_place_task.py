#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time

import actionlib
import rospy
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

from allmovebase.dog_arm_task_client import DogArmTaskClient
from allmovebase.task_budget import TaskBudget


class PickPlaceTask:
    def __init__(self):
        rospy.init_node("pick_place_task", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.goals_yaml = rospy.get_param(
            "~goals_yaml", os.path.join(os.path.dirname(__file__), "../config/task_poses.yaml")
        )
        self.state_file = rospy.get_param(
            "~meter_state_file",
            os.path.join(os.path.dirname(__file__), "../config/meter_state.yaml"),
        )
        self.region_order = self._parse_list(rospy.get_param("~region_order", "A,B,C,D"))
        self.max_abnormal_count = int(rospy.get_param("~max_abnormal_count", 2))
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout", 60.0))
        self.task_budget_reserve_sec = float(rospy.get_param("~task_budget_reserve", 5.0))
        self.min_pick_cycle_remaining_sec = float(rospy.get_param("~min_pick_cycle_remaining", 25.0))
        self.budget = TaskBudget.from_params(default_enabled=False, default_total_sec=300.0)

        self.prepare_motion_host_enabled = self._param_bool("~prepare_motion_host", True)
        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.motion_prepare_command = rospy.get_param("~motion_prepare_command", "prepare_navigation")
        self.motion_cmd_wait_timeout_sec = float(rospy.get_param("~motion_cmd_wait_timeout", 5.0))
        self.motion_prepare_wait_sec = float(rospy.get_param("~motion_prepare_wait", 1.0))

        self.arm_cmd_topic = rospy.get_param("~arm_cmd_topic", "/dog_arm/task_cmd")
        self.arm_result_topic = rospy.get_param("~arm_result_topic", "/dog_arm/task_result")
        self.arm_base_adjust_event_topic = rospy.get_param("~arm_base_adjust_event_topic", "/dog_arm/base_adjust_event")
        self.arm_pick_command = rospy.get_param("~arm_pick_command", "pick")
        self.arm_place_command_template = rospy.get_param("~arm_place_command", "place_to_zone")
        self.arm_wait_sec = float(rospy.get_param("~arm_wait", 120.0))
        self.arm_command_required = self._param_bool("~arm_command_required", False)
        self.arm_pick_timeout_sec = float(rospy.get_param("~arm_pick_timeout", 180.0))
        self.arm_place_timeout_sec = float(rospy.get_param("~arm_place_timeout", 60.0))
        self.arm_base_adjust_settle_sec = float(rospy.get_param("~arm_base_adjust_settle", 1.0))
        self.arm_max_pick_adjust_retries = int(rospy.get_param("~arm_max_pick_adjust_retries", 1))
        self.report_topic = rospy.get_param("~report_topic", "/pick_place_report")

        self.mb_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=1)
        self.report_pub = rospy.Publisher(self.report_topic, String, queue_size=10)
        self.arm_client = DogArmTaskClient(
            task_cmd_topic=self.arm_cmd_topic,
            task_result_topic=self.arm_result_topic,
            base_adjust_event_topic=self.arm_base_adjust_event_topic,
            wait_for_connection_timeout=self.motion_cmd_wait_timeout_sec,
            base_adjust_settle_sec=self.arm_base_adjust_settle_sec,
            max_pick_adjust_retries=self.arm_max_pick_adjust_retries,
            report=self._report,
        )

        self.poses = self._load_pose_registry(self.goals_yaml)
        self.pickup_key, self.place_keys = self._load_pick_place_keys(self.goals_yaml)

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _parse_list(self, value):
        if isinstance(value, str):
            return [item.strip().upper() for item in value.split(",") if item.strip()]
        return [str(item).strip().upper() for item in value if str(item).strip()]

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def _load_pose_registry(self, path):
        data = self._load_yaml(path)
        poses = {}
        for namespace in ("goals", "waypoints"):
            values = data.get(namespace, {})
            if isinstance(values, dict):
                poses.update(values)
        return poses

    def _load_pick_place_keys(self, path):
        data = self._load_yaml(path)
        pick_place = data.get("sequences", {}).get("pick_place", {})
        pickup_key = str(pick_place.get("pickup", "pickup_pose")).strip()
        place_keys = pick_place.get("places", {}) or {}
        normalized = {}
        for region in self.region_order:
            normalized[region] = str(place_keys.get(region, "place_pose_%s" % region)).strip()
        return pickup_key, normalized

    def _load_meter_states(self):
        data = self._load_yaml(self.state_file)
        states = data.get("states", data)
        if not isinstance(states, dict):
            return {}
        return {
            str(region).strip().upper(): str(status).strip().lower()
            for region, status in states.items()
            if str(region).strip()
        }

    def _abnormal_regions(self):
        states = self._load_meter_states()
        result = []
        for region in self.region_order:
            status = states.get(region, "")
            if status and status != "normal":
                result.append(region)
        return result[: max(0, self.max_abnormal_count)]

    def _wait_for_publisher_connection(self, publisher, topic, timeout):
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
        if not self._wait_for_publisher_connection(
            self.motion_cmd_pub, self.motion_cmd_topic, self.motion_cmd_wait_timeout_sec
        ):
            rospy.logerr("no subscriber on %s", self.motion_cmd_topic)
            return False
        rospy.loginfo("preparing Lite3 motion host: %s", self.motion_prepare_command)
        self.motion_cmd_pub.publish(String(data=self.motion_prepare_command))
        self.budget.sleep(self.motion_prepare_wait_sec, reserve=self.task_budget_reserve_sec)
        return True

    def _wait_for_move_base(self):
        timeout = self.budget.cap_timeout(self.nav_timeout_sec, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.mb_client.wait_for_server(rospy.Duration(2.0)):
                return True
            rospy.loginfo("waiting for /move_base action server...")
        return False

    def _make_goal(self, pose_key):
        pose_data = self.poses.get(pose_key)
        if not pose_data:
            raise ValueError("pose %s not found in %s" % (pose_key, self.goals_yaml))
        pos = pose_data.get("position")
        ori = pose_data.get("orientation")
        if not (
            isinstance(pos, (list, tuple))
            and len(pos) == 3
            and isinstance(ori, (list, tuple))
            and len(ori) == 4
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
        rospy.loginfo("navigating to %s (%s)", label, pose_key)
        self.mb_client.send_goal(self._make_goal(pose_key))
        timeout = self.budget.cap_timeout(self.nav_timeout_sec, reserve=self.task_budget_reserve_sec)
        if timeout <= 0.0 or not self.mb_client.wait_for_result(rospy.Duration(timeout)):
            self.mb_client.cancel_goal()
            rospy.logerr("navigation to %s timed out", label)
            return False
        ok = self.mb_client.get_state() == 3
        if not ok:
            rospy.logerr("navigation to %s failed, action state=%s", label, self.mb_client.get_state())
        return ok

    def _publish_arm_command(self, command, label):
        if self.arm_command_required and not self.arm_client.wait_for_server(self.motion_cmd_wait_timeout_sec):
            rospy.logerr("no subscriber on %s for %s", self.arm_cmd_topic, label)
            return False
        if str(command).strip().split(":", 1)[0] == "pick":
            timeout = self.budget.cap_timeout(self.arm_pick_timeout_sec, reserve=self.task_budget_reserve_sec)
            return self.arm_client.pick_with_adjust_retry(label, timeout)
        timeout = self.budget.cap_timeout(self.arm_place_timeout_sec, reserve=self.task_budget_reserve_sec)
        return self.arm_client.place_to_zone(label, timeout)

    def _report(self, text):
        rospy.loginfo(text)
        self.report_pub.publish(String(data=text))

    def run(self):
        if not self._prepare_motion_host():
            return
        if not self._wait_for_move_base():
            rospy.logerr("/move_base server is not available")
            return

        abnormal_regions = self._abnormal_regions()
        if not abnormal_regions:
            self._report("pick-place skipped: no abnormal meter states found")
            return

        self._report("pick-place abnormal regions: %s" % ",".join(abnormal_regions))
        for index, region in enumerate(abnormal_regions, start=1):
            if not self.budget.check("pick-place cycle %d" % index, self.min_pick_cycle_remaining_sec):
                return
            place_key = self.place_keys.get(region)
            if not place_key:
                self._report("pick-place skipped %s: missing place pose key" % region)
                continue

            if not self._navigate_to(self.pickup_key, "pickup %d" % index):
                return
            if not self._publish_arm_command(self.arm_pick_command, "pickup %d" % index):
                return

            if not self._navigate_to(place_key, "place %s" % region):
                return
            command = self.arm_place_command_template.format(region=region, index=index)
            if not self._publish_arm_command(command, "place %s" % region):
                return

        self._report("pick-place task finished")


if __name__ == "__main__":
    try:
        PickPlaceTask().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("pick_place_task failed: %s", exc)
