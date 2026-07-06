#!/usr/bin/env python3
# coding: utf-8

import math
import os
import time

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from task_budget import TaskBudget


class ObstacleZoneTask(object):
    """避障区域任务：按顺序穿越避障区入口/中点/出口。

    任务目标:
    - 在两个随机静态雪糕筒存在时，依赖局部规划器（TEB）完成穿越
    - 对外发布任务状态和结果，供后续总状态机集成
    """

    def __init__(self):
        rospy.init_node("obstacle_zone_task", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.nav_timeout_sec = float(rospy.get_param("~nav_timeout", 35.0))
        self.task_budget_reserve_sec = float(rospy.get_param("~task_budget_reserve", 5.0))
        self.min_waypoint_remaining_sec = float(rospy.get_param("~min_waypoint_remaining", 10.0))
        self.budget = TaskBudget.from_params(default_enabled=False, default_total_sec=300.0)
        self.block_hold_seconds = float(rospy.get_param("~block_hold_seconds", 2.0))
        self.obstacle_warn_distance = float(rospy.get_param("~obstacle_warn_distance", 0.45))
        self.scan_wait_timeout_sec = float(rospy.get_param("~scan_wait_timeout", 8.0))
        self.scan_stale_timeout_sec = float(rospy.get_param("~scan_stale_timeout", 1.0))
        self.require_scan = self._param_bool("~require_scan", True)
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.prepare_motion_host_enabled = self._param_bool("~prepare_motion_host", True)
        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.motion_cmd_wait_timeout_sec = float(rospy.get_param("~motion_cmd_wait_timeout", 5.0))
        self.motion_prepare_command = rospy.get_param("~motion_prepare_command", "prepare_navigation")
        self.motion_prepare_wait_sec = float(rospy.get_param("~motion_prepare_wait", 1.0))
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.require_map = self._param_bool("~require_map", True)
        self.map_wait_timeout_sec = float(rospy.get_param("~map_wait_timeout", 30.0))
        self.require_tf = self._param_bool("~require_tf", True)
        self.tf_wait_timeout_sec = float(rospy.get_param("~tf_wait_timeout", 30.0))
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.move_base_action_name = rospy.get_param("~move_base_action", "/move_base")
        self.move_base_wait_timeout_sec = float(rospy.get_param("~move_base_wait_timeout", 90.0))
        self.move_base_retry_interval_sec = float(rospy.get_param("~move_base_retry_interval", 2.0))
        self.move_base_status_topic = rospy.get_param("~move_base_status_topic", "/move_base/status")
        self.require_move_base_status = self._param_bool("~require_move_base_status", True)
        self.move_base_status_wait_timeout_sec = float(
            rospy.get_param("~move_base_status_wait_timeout", 30.0)
        )

        self.waypoints_yaml = rospy.get_param("~waypoints_yaml", "")
        if not self.waypoints_yaml:
            raise rospy.ROSInitException("缺少 ~waypoints_yaml")
        if not os.path.isabs(self.waypoints_yaml):
            self.waypoints_yaml = os.path.abspath(self.waypoints_yaml)
        if not os.path.exists(self.waypoints_yaml):
            raise rospy.ROSInitException("waypoints_yaml 不存在: {}".format(self.waypoints_yaml))

        order_param = rospy.get_param("~waypoint_order", "obs_entry,obs_mid,obs_exit")
        if isinstance(order_param, str):
            self.waypoint_order = [x.strip() for x in order_param.split(",") if x.strip()]
        else:
            self.waypoint_order = [str(x).strip() for x in order_param if str(x).strip()]
        if not self.waypoint_order:
            raise rospy.ROSInitException("waypoint_order 为空")

        self._latest_scan_min = float("inf")
        self._latest_scan_time = 0.0
        self._close_obstacle_since = None

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._on_scan, queue_size=1)
        self.report_pub = rospy.Publisher("/obstacle_task/report", String, queue_size=10)
        self.done_pub = rospy.Publisher("/obstacle_task/succeeded", Bool, queue_size=1, latch=True)
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.mb_client = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)

        self.waypoints = self._load_waypoints(self.waypoints_yaml, self.waypoint_order)
        rospy.loginfo("obstacle_zone_task ready, order=%s", self.waypoint_order)

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.02]
        if valid:
            self._latest_scan_min = min(valid)
            self._latest_scan_time = time.time()

    def _wait_for_scan(self):
        if not self.require_scan:
            return True

        self._publish_report("waiting for obstacle scan: {}".format(self.scan_topic))
        t0 = time.time()
        timeout = self.budget.cap_timeout(self.scan_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._latest_scan_time > 0:
                self._publish_report("scan ready, min_scan={:.3f}m".format(self._latest_scan_min))
                return True
            if timeout <= 0.0 or (time.time() - t0) > timeout:
                self._publish_report(
                    "scan wait timeout on {} after {:.1f}s; abort obstacle task".format(
                        self.scan_topic, self.scan_wait_timeout_sec
                    )
                )
                return False
            if self.require_scan and self._latest_scan_time > 0:
                scan_age = time.time() - self._latest_scan_time
                if scan_age > self.scan_stale_timeout_sec:
                    rospy.logwarn("obstacle scan is stale: %.2fs", scan_age)

            rate.sleep()

        return False

    def _load_waypoints(self, yaml_path, order):
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        pool = data.get("waypoints", {})
        sequences = data.get("sequences", {})
        if len(order) == 1 and order[0] in sequences:
            order = [str(x).strip() for x in sequences[order[0]] if str(x).strip()]

        out = []
        for name in order:
            wp = pool.get(name)
            if not wp:
                rospy.logwarn("waypoints 缺少: %s", name)
                continue
            pos = wp.get("position")
            ori = wp.get("orientation")
            if not (isinstance(pos, (list, tuple)) and len(pos) == 3 and isinstance(ori, (list, tuple)) and len(ori) == 4):
                rospy.logwarn("waypoint %s 格式错误", name)
                continue
            try:
                pos = [float(v) for v in pos]
                ori = [float(v) for v in ori]
            except (TypeError, ValueError):
                rospy.logwarn("waypoint %s has non-numeric values, skipped", name)
                continue
            out.append({"name": name, "position": pos, "orientation": ori})

        if not out:
            raise rospy.ROSInitException("未加载到有效 waypoint")
        return out

    def _publish_report(self, text):
        rospy.loginfo(text)
        self.report_pub.publish(String(data=text))

    def _wait_for_map(self):
        if not self.require_map:
            return True
        self._publish_report("waiting for map on {}".format(self.map_topic))
        try:
            timeout = self.budget.cap_timeout(self.map_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
            rospy.wait_for_message(self.map_topic, OccupancyGrid, timeout=timeout)
            self._publish_report("map ready: {}".format(self.map_topic))
            return True
        except rospy.ROSException:
            self._publish_report(
                "map wait timeout on {} after {:.1f}s".format(
                    self.map_topic, self.map_wait_timeout_sec
                )
            )
            return False

    def _wait_for_tf(self):
        if not self.require_tf:
            return True
        self._publish_report(
            "waiting for TF {} -> {}".format(self.frame_id, self.base_frame_id)
        )
        timeout = self.budget.cap_timeout(self.tf_wait_timeout_sec, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        last_warn = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                self.tf_listener.waitForTransform(
                    self.frame_id,
                    self.base_frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0),
                )
                self._publish_report(
                    "TF ready: {} -> {}".format(self.frame_id, self.base_frame_id)
                )
                return True
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                now = time.time()
                if now - last_warn > 2.0:
                    self._publish_report(
                        "still waiting for TF {} -> {}: {}".format(
                            self.frame_id, self.base_frame_id, exc
                        )
                    )
                    last_warn = now
        self._publish_report(
            "TF wait timeout: {} -> {} after {:.1f}s".format(
                self.frame_id, self.base_frame_id, self.tf_wait_timeout_sec
            )
        )
        return False

    def _wait_for_move_base_status(self):
        if not self.require_move_base_status:
            return True
        self._publish_report("waiting for move_base status on {}".format(self.move_base_status_topic))
        try:
            rospy.wait_for_message(
                self.move_base_status_topic,
                GoalStatusArray,
                timeout=self.budget.cap_timeout(
                    self.move_base_status_wait_timeout_sec,
                    reserve=self.task_budget_reserve_sec,
                ),
            )
            self._publish_report("move_base status ready: {}".format(self.move_base_status_topic))
            return True
        except rospy.ROSException:
            self._publish_report(
                "move_base status wait timeout on {} after {:.1f}s".format(
                    self.move_base_status_topic,
                    self.move_base_status_wait_timeout_sec,
                )
            )
            return False

    def _wait_for_move_base_server(self):
        self._publish_report(
            "waiting for {} action server, timeout={:.1f}s".format(
                self.move_base_action_name, self.move_base_wait_timeout_sec
            )
        )
        start = time.time()
        attempt = 0
        while not rospy.is_shutdown():
            elapsed = time.time() - start
            remaining = self.move_base_wait_timeout_sec - elapsed
            remaining = min(remaining, self.budget.cap_timeout(remaining, reserve=self.task_budget_reserve_sec))
            if remaining <= 0.0:
                self._publish_report(
                    "{} action server not ready after {:.1f}s".format(
                        self.move_base_action_name, self.move_base_wait_timeout_sec
                    )
                )
                return False

            attempt += 1
            wait_slice = min(self.move_base_retry_interval_sec, remaining)
            if self.mb_client.wait_for_server(rospy.Duration(wait_slice)):
                self._publish_report("{} action server ready".format(self.move_base_action_name))
                return True
            self._publish_report(
                "still waiting for {} action server, attempt={}, elapsed={:.1f}s".format(
                    self.move_base_action_name, attempt, elapsed + wait_slice
                )
            )
        return False

    def _wait_for_navigation_stack(self):
        if not self._wait_for_map():
            return False
        if not self._wait_for_scan():
            return False
        if not self._wait_for_tf():
            return False
        if not self._wait_for_move_base_status():
            return False
        return self._wait_for_move_base_server()

    def _prepare_motion_host(self):
        if not self.prepare_motion_host_enabled:
            return True

        self._publish_report("preparing Lite3 motion host before obstacle navigation")
        t0 = time.time()
        timeout = self.budget.cap_timeout(
            self.motion_cmd_wait_timeout_sec,
            reserve=self.task_budget_reserve_sec,
        )
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.motion_cmd_pub.get_num_connections() == 0:
            if timeout <= 0.0 or (time.time() - t0) > timeout:
                self._publish_report(
                    "lite3 motion command node not connected on {} after {:.1f}s".format(
                        self.motion_cmd_topic, self.motion_cmd_wait_timeout_sec
                    )
                )
                return False
            rate.sleep()

        self._publish_report("Lite3 motion command: {}".format(self.motion_prepare_command))
        self.motion_cmd_pub.publish(String(data=self.motion_prepare_command))
        self.budget.sleep(self.motion_prepare_wait_sec, reserve=self.task_budget_reserve_sec)
        return True

    def _build_goal(self, wp):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position.x = float(wp["position"][0])
        goal.target_pose.pose.position.y = float(wp["position"][1])
        goal.target_pose.pose.position.z = float(wp["position"][2])
        goal.target_pose.pose.orientation.x = float(wp["orientation"][0])
        goal.target_pose.pose.orientation.y = float(wp["orientation"][1])
        goal.target_pose.pose.orientation.z = float(wp["orientation"][2])
        goal.target_pose.pose.orientation.w = float(wp["orientation"][3])
        return goal

    def _navigate_one(self, wp):
        name = wp["name"]
        if not self.budget.check("obstacle waypoint %s" % name, self.min_waypoint_remaining_sec):
            return False
        self._publish_report("前往避障点: {}".format(name))

        goal = self._build_goal(wp)
        self.mb_client.send_goal(goal)

        t0 = time.time()
        rate = rospy.Rate(20)
        self._close_obstacle_since = None

        while not rospy.is_shutdown():
            state = self.mb_client.get_state()
            if state == 3:
                self._publish_report("到达避障点: {}".format(name))
                return True
            if state in (4, 5, 6, 7, 8, 9):
                self._publish_report("避障点 {} 导航失败，state={}".format(name, state))
                return False

            elapsed = time.time() - t0
            timeout = self.budget.cap_timeout(self.nav_timeout_sec, reserve=self.task_budget_reserve_sec)
            if timeout <= 0.0 or elapsed > timeout:
                self.mb_client.cancel_goal()
                self._publish_report("避障点 {} 导航超时({:.1f}s)".format(name, self.nav_timeout_sec))
                return False

            if self._latest_scan_time > 0 and self._latest_scan_min < self.obstacle_warn_distance:
                if self._close_obstacle_since is None:
                    self._close_obstacle_since = time.time()
                elif (time.time() - self._close_obstacle_since) > self.block_hold_seconds:
                    rospy.logwarn("近障持续 %.2fs, min_scan=%.3f", time.time() - self._close_obstacle_since, self._latest_scan_min)
            else:
                self._close_obstacle_since = None

            rate.sleep()

        return False

    def run(self):
        self.done_pub.publish(Bool(data=False))
        self._publish_report("开始执行避障区域任务")

        if not self._wait_for_navigation_stack():
            self.done_pub.publish(Bool(data=False))
            return

        if not self._prepare_motion_host():
            self.done_pub.publish(Bool(data=False))
            return

        for wp in self.waypoints:
            ok = self._navigate_one(wp)
            if not ok:
                self._publish_report("避障任务失败，终止")
                self.done_pub.publish(Bool(data=False))
                return

        self._publish_report("避障任务完成：已通过避障区域")
        self.done_pub.publish(Bool(data=True))


if __name__ == "__main__":
    try:
        node = ObstacleZoneTask()
        node.run()
    except rospy.ROSInitException as exc:
        rospy.logerr(str(exc))
    except Exception as exc:
        rospy.logerr("obstacle_zone_task exception: %s", exc)

