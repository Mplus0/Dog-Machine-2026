#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import String

from allmovebase.task_budget import TaskBudget


class HardcodedMotion:
    def __init__(self):
        rospy.init_node("hardcoded_motion")

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.motion_cmd_topic = rospy.get_param("~motion_cmd_topic", "/lite3_motion_cmd")
        self.motion_cmd_pub = rospy.Publisher(self.motion_cmd_topic, String, queue_size=1)
        self.inspect_trigger_pub = rospy.Publisher(
            rospy.get_param("~meter_trigger_topic", "/meter_inspect_trigger"),
            String,
            queue_size=1,
        )
        self.meter_result_topic = rospy.get_param("~meter_result_topic", "/meter_status")
        self.meter_ready_topic = rospy.get_param("~meter_ready_topic", "/meter_inspection_ready")
        self.meter_ready_timeout = float(rospy.get_param("~meter_ready_timeout", 120.0))
        self.inspect_timeout = float(rospy.get_param("~meter_timeout", 30.0))
        self.enable_meter_inspection = self._param_bool("~enable_meter_inspection", True)
        self.prepare_motion_host_enabled = self._param_bool("~prepare_motion_host", True)
        self.motion_prepare_command = rospy.get_param("~motion_prepare_command", "prepare_hardcoded_motion")
        self.motion_cmd_wait_timeout = float(rospy.get_param("~motion_cmd_wait_timeout", 5.0))
        self.motion_prepare_wait = float(rospy.get_param("~motion_prepare_wait", 1.0))
        self.inspection_view_command = rospy.get_param("~inspection_view_command", "inspection_view_pose")
        self.navigation_view_command = rospy.get_param("~navigation_view_command", "navigation_view_pose")
        self.require_cmd_vel_subscriber = self._param_bool("~require_cmd_vel_subscriber", True)
        self.cmd_vel_wait_timeout = float(rospy.get_param("~cmd_vel_wait_timeout", 8.0))
        self.cmd_vel_publish_hz = float(rospy.get_param("~cmd_vel_publish_hz", 10.0))
        self.task_budget_reserve_sec = float(rospy.get_param("~task_budget_reserve", 5.0))
        self.min_motion_segment_remaining_sec = float(rospy.get_param("~min_motion_segment_remaining", 5.0))
        self.min_inspection_remaining_sec = float(rospy.get_param("~min_inspection_remaining", 12.0))
        self.budget = TaskBudget.from_params(default_enabled=False, default_total_sec=300.0)

        self.linear_speed = float(rospy.get_param("~linear_speed", 0.5))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.5))
        self.turn_angle_deg = float(rospy.get_param("~turn_angle_deg", 90.0))
        self.turn_duration_scale = float(rospy.get_param("~turn_duration_scale", 1.5))
        self.settle_after_motion = float(rospy.get_param("~settle_after_motion", 0.2))
        self.inspect_pose_settle = float(rospy.get_param("~inspect_pose_settle", 1.0))
        self.post_inspection_hold = float(rospy.get_param("~post_inspection_hold", 4.0))
        self.closed_loop_motion = self._param_bool("~closed_loop_motion", False)
        self.closed_loop_straight = self._param_bool("~closed_loop_straight", True)
        self.closed_loop_turn = self._param_bool("~closed_loop_turn", True)
        self.closed_loop_require_feedback = self._param_bool("~closed_loop_require_feedback", False)
        self.odom_topic = rospy.get_param("~closed_loop_odom_topic", "/leg_odom2")
        self.odom_timeout = float(rospy.get_param("~closed_loop_odom_timeout", 1.0))
        self.feedback_wait_timeout = float(rospy.get_param("~closed_loop_feedback_wait_timeout", 5.0))
        self.distance_tolerance = float(rospy.get_param("~closed_loop_distance_tolerance", 0.05))
        self.turn_tolerance_rad = math.radians(float(rospy.get_param("~closed_loop_turn_tolerance_deg", 4.0)))
        self.closed_loop_max_time_scale = float(rospy.get_param("~closed_loop_max_time_scale", 2.5))
        self.closed_loop_max_time_margin = float(rospy.get_param("~closed_loop_max_time_margin", 1.0))
        self.closed_loop_min_speed_ratio = float(rospy.get_param("~closed_loop_min_speed_ratio", 0.35))
        self.closed_loop_slowdown_distance = float(rospy.get_param("~closed_loop_slowdown_distance", 0.25))
        self.closed_loop_slowdown_angle_rad = math.radians(
            float(rospy.get_param("~closed_loop_slowdown_angle_deg", 25.0))
        )

        self.initial_turn_to_y_pos = rospy.get_param("~initial_turn_to_y_pos", "none")
        self.segment_distances = {
            "obs_end_to_rec_pose_1": float(rospy.get_param("~obs_end_to_rec_pose_1_distance", 1.25)),
            "rec_pose_1_to_rec_pose_2": float(rospy.get_param("~rec_pose_1_to_rec_pose_2_distance", 2.5)),
            "half_loop_leg_1_after_rec_pose_2": float(rospy.get_param("~half_loop_leg_1_distance", 0.45)),
            "half_loop_leg_2_cross_lane": float(rospy.get_param("~half_loop_leg_2_distance", 2.0)),
            "half_loop_leg_3_to_rec_pose_4": float(rospy.get_param("~half_loop_leg_3_distance", 1.0)),
            "rec_pose_4_to_rec_pose_3": float(rospy.get_param("~rec_pose_4_to_rec_pose_3_distance", 2.5)),
        }
        self.inspect_turns = {
            "rec_pose_1": rospy.get_param("~rec_pose_1_inspect_turn", "right"),
            "rec_pose_2": rospy.get_param("~rec_pose_2_inspect_turn", "right"),
            "rec_pose_4": rospy.get_param("~rec_pose_4_inspect_turn", "right"),
            "rec_pose_3": rospy.get_param("~rec_pose_3_inspect_turn", "right"),
        }

        self.latest_meter_result = None
        self.expected_meter_trigger = None
        self.result_event = threading.Event()
        self.odom_lock = threading.Lock()
        self.latest_odom = None
        self.latest_odom_time = None
        rospy.Subscriber(self.meter_result_topic, String, self._meter_result_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)

        self.budget.sleep(1.0, reserve=self.task_budget_reserve_sec)
        rospy.loginfo("hardcoded_motion ready, meter_inspection=%s, closed_loop=%s, odom=%s",
                      self.enable_meter_inspection, self.closed_loop_motion, self.odom_topic)

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _meter_result_callback(self, msg):
        data = str(msg.data).strip()
        parts = [item.strip() for item in data.split(",")]
        if len(parts) == 3 and self.expected_meter_trigger:
            if parts[0] != self.expected_meter_trigger:
                rospy.logwarn(
                    "ignore stale meter result for %s while waiting for %s: %s",
                    parts[0],
                    self.expected_meter_trigger,
                    data,
                )
                return
        self.latest_meter_result = data
        self.result_event.set()

    def _odom_callback(self, msg):
        with self.odom_lock:
            self.latest_odom = msg
            self.latest_odom_time = rospy.Time.now()

    def _get_odom_pose(self):
        with self.odom_lock:
            if self.latest_odom is None or self.latest_odom_time is None:
                return None
            age = (rospy.Time.now() - self.latest_odom_time).to_sec()
            if age > self.odom_timeout:
                return None
            pose = self.latest_odom.pose.pose
            return (
                pose.position.x,
                pose.position.y,
                self._yaw_from_quaternion(pose.orientation),
            )

    def _wait_for_odom_pose(self):
        timeout = self.budget.cap_timeout(self.feedback_wait_timeout, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._get_odom_pose()
            if pose is not None:
                return pose
            self.budget.sleep(0.05, reserve=self.task_budget_reserve_sec)
        return None

    def _yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _scaled_speed(self, base_speed, remaining, slowdown_window):
        if slowdown_window <= 0.0 or remaining >= slowdown_window:
            return base_speed
        ratio = max(self.closed_loop_min_speed_ratio, remaining / slowdown_window)
        return base_speed * ratio

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        rate = rospy.Rate(self.cmd_vel_publish_hz)
        duration = self.budget.cap_timeout(duration, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + max(0.0, duration)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.stop()

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.budget.sleep(self.settle_after_motion, reserve=self.task_budget_reserve_sec)

    def move_until_distance(self, linear, target_distance, open_loop_duration):
        start_pose = self._wait_for_odom_pose()
        if start_pose is None:
            rospy.logwarn("closed-loop straight feedback unavailable on %s", self.odom_topic)
            if self.closed_loop_require_feedback:
                self.stop()
                return False
            rospy.logwarn("falling back to timed straight motion")
            self.move(linear, 0.0, open_loop_duration)
            return True

        target = abs(target_distance)
        if target <= self.distance_tolerance:
            self.stop()
            return True

        start_x, start_y, _ = start_pose
        max_duration = max(0.1, open_loop_duration * self.closed_loop_max_time_scale + self.closed_loop_max_time_margin)
        max_duration = self.budget.cap_timeout(max_duration, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + max_duration
        rate = rospy.Rate(self.cmd_vel_publish_hz)
        twist = Twist()

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._get_odom_pose()
            if pose is None:
                rospy.logwarn("closed-loop straight lost odom feedback")
                if self.closed_loop_require_feedback:
                    self.stop()
                    return False
                break

            dx = pose[0] - start_x
            dy = pose[1] - start_y
            traveled = math.hypot(dx, dy)
            remaining = target - traveled
            if remaining <= self.distance_tolerance:
                rospy.loginfo("closed-loop straight reached %.3fm / %.3fm", traveled, target)
                self.stop()
                return True

            twist.linear.x = self._scaled_speed(linear, remaining, self.closed_loop_slowdown_distance)
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            rate.sleep()

        rospy.logwarn("closed-loop straight timeout, stop after target %.3fm", target)
        self.stop()
        return False

    def move_until_turn(self, angular, target_angle, open_loop_duration):
        start_pose = self._wait_for_odom_pose()
        if start_pose is None:
            rospy.logwarn("closed-loop turn feedback unavailable on %s", self.odom_topic)
            if self.closed_loop_require_feedback:
                self.stop()
                return False
            rospy.logwarn("falling back to timed turn motion")
            self.move(0.0, angular, open_loop_duration)
            return True

        target = abs(target_angle)
        if target <= self.turn_tolerance_rad:
            self.stop()
            return True

        start_yaw = start_pose[2]
        turn_sign = 1.0 if angular >= 0.0 else -1.0
        max_duration = max(0.1, open_loop_duration * self.closed_loop_max_time_scale + self.closed_loop_max_time_margin)
        max_duration = self.budget.cap_timeout(max_duration, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + max_duration
        rate = rospy.Rate(self.cmd_vel_publish_hz)
        twist = Twist()

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._get_odom_pose()
            if pose is None:
                rospy.logwarn("closed-loop turn lost odom feedback")
                if self.closed_loop_require_feedback:
                    self.stop()
                    return False
                break

            delta = self._normalize_angle(pose[2] - start_yaw) * turn_sign
            remaining = target - delta
            if remaining <= self.turn_tolerance_rad:
                rospy.loginfo("closed-loop turn reached %.1fdeg / %.1fdeg",
                              math.degrees(max(delta, 0.0)), math.degrees(target))
                self.stop()
                return True

            twist.linear.x = 0.0
            twist.angular.z = self._scaled_speed(angular, remaining, self.closed_loop_slowdown_angle_rad)
            self.cmd_pub.publish(twist)
            rate.sleep()

        rospy.logwarn("closed-loop turn timeout, stop after target %.1fdeg", math.degrees(target))
        self.stop()
        return False

    def _wait_for_cmd_vel_subscriber(self):
        if not self.require_cmd_vel_subscriber:
            return True
        start = time.time()
        timeout = self.budget.cap_timeout(self.cmd_vel_wait_timeout, reserve=self.task_budget_reserve_sec)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.cmd_pub.get_num_connections() == 0:
            if timeout <= 0.0 or time.time() - start > timeout:
                rospy.logerr("no subscriber on %s after %.1fs", self.cmd_vel_topic, self.cmd_vel_wait_timeout)
                return False
            rate.sleep()
        rospy.loginfo("%s subscriber ready, connections=%d", self.cmd_vel_topic, self.cmd_pub.get_num_connections())
        return True

    def _wait_for_motion_cmd_subscriber(self):
        start = time.time()
        timeout = self.budget.cap_timeout(self.motion_cmd_wait_timeout, reserve=self.task_budget_reserve_sec)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.motion_cmd_pub.get_num_connections() == 0:
            if timeout <= 0.0 or time.time() - start > timeout:
                rospy.logerr("no subscriber on %s", self.motion_cmd_topic)
                return False
            rate.sleep()
        return True

    def publish_motion_command(self, command, wait_after=0.5):
        if not self._wait_for_motion_cmd_subscriber():
            return False
        rospy.loginfo("hardcoded motion command: %s", command)
        self.motion_cmd_pub.publish(String(data=command))
        self.budget.sleep(wait_after, reserve=self.task_budget_reserve_sec)
        return True

    def prepare_motion_host(self):
        if not self.prepare_motion_host_enabled:
            return self._wait_for_cmd_vel_subscriber()
        if not self.publish_motion_command(self.motion_prepare_command, wait_after=self.motion_prepare_wait):
            return False
        self.stop()
        return self._wait_for_cmd_vel_subscriber()

    def drive_straight(self, segment_name):
        distance = self.segment_distances[segment_name]
        duration = abs(distance) / max(abs(self.linear_speed), 1.0e-6)
        min_remaining = max(self.min_motion_segment_remaining_sec, duration + self.task_budget_reserve_sec)
        if not self.budget.check("hardcoded segment %s" % segment_name, min_remaining):
            return False
        linear = self.linear_speed if distance >= 0.0 else -self.linear_speed
        if self.closed_loop_motion and self.closed_loop_straight:
            rospy.loginfo("hardcoded segment %s: closed-loop straight %.2fm at %.2fm/s",
                          segment_name, distance, linear)
            return self.move_until_distance(linear, distance, duration)
        else:
            rospy.loginfo("hardcoded segment %s: timed straight %.2fm, %.2fs at %.2fm/s",
                          segment_name, distance, duration, linear)
            self.move(linear, 0.0, duration)
            return True

    def turn(self, direction, label="turn_90"):
        direction = str(direction).strip().lower()
        if direction in ("none", "skip", "0", ""):
            rospy.loginfo("hardcoded %s skipped", label)
            return True
        if direction not in ("left", "right"):
            rospy.logwarn("unknown turn direction %s for %s, skipped", direction, label)
            return True
        angular = self.turn_speed if direction == "left" else -self.turn_speed
        target_angle = abs(self.turn_angle_deg) * math.pi / 180.0
        duration = target_angle / max(abs(self.turn_speed), 1.0e-6)
        duration *= max(self.turn_duration_scale, 0.0)
        min_remaining = max(self.min_motion_segment_remaining_sec, duration + self.task_budget_reserve_sec)
        if not self.budget.check("hardcoded turn %s" % label, min_remaining):
            return False
        if self.closed_loop_motion and self.closed_loop_turn:
            rospy.loginfo("hardcoded %s: closed-loop %s %.1fdeg at %.2frad/s, max scale=%.2f",
                          label, direction, self.turn_angle_deg, angular, self.closed_loop_max_time_scale)
            return self.move_until_turn(angular, target_angle, duration)
        else:
            rospy.loginfo("hardcoded %s: timed %s %.1fdeg %.2fs at %.2frad/s, scale=%.2f",
                          label, direction, self.turn_angle_deg, duration, angular, self.turn_duration_scale)
            self.move(0.0, angular, duration)
            return True

    def opposite_turn(self, direction):
        direction = str(direction).strip().lower()
        if direction == "left":
            return "right"
        if direction == "right":
            return "left"
        return "none"

    def enter_inspection_view(self):
        return self.publish_motion_command(self.inspection_view_command, wait_after=self.inspect_pose_settle)

    def exit_inspection_view(self):
        return self.publish_motion_command(self.navigation_view_command, wait_after=0.5)

    def recognize_and_speak(self, target_id):
        rospy.loginfo("meter inspection point %s", target_id)
        if not self.enable_meter_inspection:
            self.budget.sleep(1.0, reserve=self.task_budget_reserve_sec)
            return None

        if not self.budget.check("meter inspection %s" % target_id, self.min_inspection_remaining_sec):
            return None
        if not self.wait_for_meter_ready():
            rospy.logwarn("meter inspection point %s skipped: meter node not ready", target_id)
            return None

        self.result_event.clear()
        self.latest_meter_result = None
        self.expected_meter_trigger = str(target_id)
        self.inspect_trigger_pub.publish(String(data=self.expected_meter_trigger))

        timeout = self.budget.cap_timeout(self.inspect_timeout, reserve=self.task_budget_reserve_sec)
        if timeout > 0.0 and self.result_event.wait(timeout):
            rospy.loginfo("meter inspection point %s result: %s",
                          target_id, self.latest_meter_result)
            self.expected_meter_trigger = None
            return self.latest_meter_result

        rospy.logwarn("meter inspection point %s timed out after %.1fs",
                      target_id, self.inspect_timeout)
        self.expected_meter_trigger = None
        return None

    def wait_for_meter_ready(self):
        if not self.meter_ready_topic:
            return True
        timeout = self.budget.cap_timeout(self.meter_ready_timeout, reserve=self.task_budget_reserve_sec)
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                ready = rospy.wait_for_message(self.meter_ready_topic, Bool, timeout=1.0)
                if ready.data:
                    return True
                self.budget.sleep(0.2, reserve=self.task_budget_reserve_sec)
            except rospy.ROSException:
                pass
        return False

    def inspect_point(self, point_name):
        turn_direction = self.inspect_turns[point_name]
        if not self.turn(turn_direction, "%s_face_meter" % point_name):
            return False
        self.stop()
        if self.enter_inspection_view():
            self.recognize_and_speak(point_name)
            if self.post_inspection_hold > 0.0:
                self.budget.sleep(self.post_inspection_hold, reserve=self.task_budget_reserve_sec)
        self.exit_inspection_view()
        return self.turn(self.opposite_turn(turn_direction), "%s_restore_route_heading" % point_name)

    def run(self):
        if not self.prepare_motion_host():
            return

        # Route assumption:
        # obstacle end -> rec_pose_1 -> rec_pose_2 along +Y,
        # then a small half-loop: straight, right, straight, right, straight,
        # then rec_pose_4 -> rec_pose_3 along -Y.
        if not self.turn(self.initial_turn_to_y_pos, "initial_turn_to_y_pos"):
            return
        if not self.drive_straight("obs_end_to_rec_pose_1"):
            return
        if not self.inspect_point("rec_pose_1"):
            return

        if not self.drive_straight("rec_pose_1_to_rec_pose_2"):
            return
        if not self.inspect_point("rec_pose_2"):
            return

        if not self.drive_straight("half_loop_leg_1_after_rec_pose_2"):
            return
        if not self.turn("right", "half_loop_right_turn_1"):
            return
        if not self.drive_straight("half_loop_leg_2_cross_lane"):
            return
        if not self.turn("right", "half_loop_right_turn_2"):
            return
        if not self.drive_straight("half_loop_leg_3_to_rec_pose_4"):
            return
        if not self.inspect_point("rec_pose_4"):
            return

        if not self.drive_straight("rec_pose_4_to_rec_pose_3"):
            return
        if not self.inspect_point("rec_pose_3"):
            return

        rospy.loginfo("hardcoded route finished")
        self.stop()


if __name__ == "__main__":
    try:
        HardcodedMotion().run()
    except rospy.ROSInterruptException:
        pass
