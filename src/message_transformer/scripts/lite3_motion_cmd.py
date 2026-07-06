#!/usr/bin/env python3
# coding: utf-8

import time

import rospy
from geometry_msgs.msg import Twist
from message_transformer.msg import SimpleCMD
from std_msgs.msg import Int32, String


class Lite3Protocol(object):
    """Constants copied from the Lite3 motion host UDP interface manual."""

    TYPE_SIMPLE = 0

    STAND_LIE_TOGGLE = 0x21010202
    SOFT_ESTOP = 0x21020C0E
    ZERO_POSITION = 0x21010C05

    SPOT_MODE = 0x21010D05
    MOVE_MODE = 0x21010D06
    AUTO_MODE = 0x21010C03
    MANUAL_MODE = 0x21010C02

    AXIS_ROLL_OR_SIDE = 0x21010131
    AXIS_PITCH_OR_FORWARD = 0x21010130
    AXIS_HEIGHT = 0x21010102
    AXIS_YAW_OR_TURN = 0x21010135

    GAIT_FLAT_LOW = 0x21010300
    GAIT_FLAT_MEDIUM = 0x21010307
    GAIT_FLAT_HIGH = 0x21010303
    GAIT_CRAWL_TOGGLE = 0x21010406
    GAIT_GRIP_OBSTACLE = 0x21010402
    GAIT_GENERAL_OBSTACLE = 0x21010401
    GAIT_HIGH_STEP = 0x21010407

    ACTION_TWIST_BODY = 0x21010204
    ACTION_FLIP = 0x21010205
    ACTION_SPACE_STEP = 0x2101030C
    ACTION_BACKFLIP = 0x21010502
    ACTION_GREET = 0x21010507
    ACTION_FORWARD_JUMP = 0x2101050B
    ACTION_TWIST_JUMP = 0x2101020D

    CONTINUOUS_MOTION = 0x21010C06
    CONTINUOUS_MOTION_ON = -1
    CONTINUOUS_MOTION_OFF = 2

    SPEAKER = 0x2101030D
    SPEAKER_OFF = 0
    SPEAKER_ON = 1
    SPEAKER_QUERY = 2

    VOICE_COMMAND = 0x21010C0A
    VOICE_STAND = 1
    VOICE_SIT = 2
    VOICE_FORWARD = 3
    VOICE_BACKWARD = 4
    VOICE_LEFT = 5
    VOICE_RIGHT = 6
    VOICE_STOP = 7
    VOICE_LOOK_DOWN = 8
    VOICE_LOOK_UP = 9
    VOICE_LOOK_LEFT = 11
    VOICE_LOOK_RIGHT = 12
    VOICE_TURN_LEFT_90 = 13
    VOICE_TURN_RIGHT_90 = 14
    VOICE_TURN_BACK_180 = 15
    VOICE_GREET = 22

    AI_OPTION = 0x21012109
    AI_DISABLE_ALL = 0x00
    AI_OBSTACLE_STOP = 0x20
    AI_FOLLOW = 0xC0

    BASIC_LIE_DOWN = 1
    BASIC_PREPARE_STAND = 4
    BASIC_STANDING_UP = 5
    BASIC_FORCE_STAND = 6
    BASIC_LYING_DOWN = 7
    BASIC_LOST_CONTROL_PROTECT = 8
    BASIC_ATTITUDE_ADJUST = 9
    BASIC_FLIPPING = 11
    BASIC_ZERO_POSITION = 17
    BASIC_BACKFLIPPING = 18
    BASIC_GREETING = 20

    GAIT_STATE_FLAT_LOW = 0
    GAIT_STATE_GENERAL_OBSTACLE = 2
    GAIT_STATE_FLAT_MEDIUM = 4
    GAIT_STATE_FLAT_HIGH = 5
    GAIT_STATE_GRIP_OBSTACLE = 6
    GAIT_STATE_SPACE_STEP = 12
    GAIT_STATE_HIGH_STEP = 13


class Lite3MotionCmd(object):
    def __init__(self):
        rospy.init_node("lite3_motion_cmd", anonymous=False)

        self.command_topic = rospy.get_param("~command_topic", "/lite3_motion_cmd")
        self.robot_basic_state_topic = rospy.get_param("~robot_basic_state_topic", "/lite3/robot_basic_state")
        self.robot_gait_state_topic = rospy.get_param("~robot_gait_state_topic", "/lite3/robot_gait_state")
        self.robot_motion_state_topic = rospy.get_param("~robot_motion_state_topic", "/lite3/robot_motion_state")

        self.robot_state_timeout_sec = float(rospy.get_param("~robot_state_timeout", 2.0))
        self.mode_switch_timeout_sec = float(rospy.get_param("~mode_switch_timeout", 3.0))
        self.mode_switch_retry_count = int(rospy.get_param("~mode_switch_retry_count", 3))
        self.stand_timeout_sec = float(rospy.get_param("~stand_timeout", 8.0))
        self.stand_settle_sec = float(rospy.get_param("~stand_settle_sec", 1.0))

        self.standing_basic_states = self._load_int_list(
            "~standing_basic_states", [Lite3Protocol.BASIC_FORCE_STAND, Lite3Protocol.BASIC_ATTITUDE_ADJUST]
        )
        self.lie_basic_states = self._load_int_list("~lie_basic_states", [Lite3Protocol.BASIC_LIE_DOWN])
        self.spot_basic_states = self._load_int_list(
            "~spot_basic_states", [Lite3Protocol.BASIC_FORCE_STAND, Lite3Protocol.BASIC_ATTITUDE_ADJUST]
        )
        self.move_basic_states = self._load_int_list(
            "~move_basic_states", [Lite3Protocol.BASIC_FORCE_STAND, Lite3Protocol.BASIC_ATTITUDE_ADJUST]
        )

        self.default_prepare_gait = rospy.get_param("~default_prepare_gait", "flat_low_gait")
        self.enter_move_mode_after_stand = self._load_bool("~enter_move_mode_after_stand", False)

        self.stop_publish_hz = float(rospy.get_param("~stop_publish_hz", 10.0))
        self.stop_duration_sec = float(rospy.get_param("~stop_duration_sec", 1.0))
        self.command_gap_sec = float(rospy.get_param("~command_gap", 0.15))
        self.view_pose_step_sleep_sec = float(rospy.get_param("~view_pose_step_sleep", 0.5))
        self.view_pose_pitch_repeat_count = int(rospy.get_param("~view_pose_pitch_repeat_count", 3))
        self.inspection_pitch_value = int(rospy.get_param("~inspection_pitch_value", -6553))
        self.navigation_pitch_value = int(rospy.get_param("~navigation_pitch_value", 0))
        self.low_pose_height_value = int(rospy.get_param("~low_pose_height_value", -20000))
        self.normal_height_value = int(rospy.get_param("~normal_height_value", 0))

        self._robot_basic_state = None
        self._robot_gait_state = None
        self._robot_motion_state = None
        self._robot_state_time = 0.0

        self.simple_pub = rospy.Publisher("/simple_cmd", SimpleCMD, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.command_sub = rospy.Subscriber(self.command_topic, String, self._on_command, queue_size=10)
        self.basic_state_sub = rospy.Subscriber(
            self.robot_basic_state_topic, Int32, self._on_robot_basic_state, queue_size=10
        )
        self.gait_state_sub = rospy.Subscriber(
            self.robot_gait_state_topic, Int32, self._on_robot_gait_state, queue_size=10
        )
        self.motion_state_sub = rospy.Subscriber(
            self.robot_motion_state_topic, Int32, self._on_robot_motion_state, queue_size=10
        )

        self._build_dispatch_table()
        rospy.loginfo("lite3_motion_cmd ready on %s", self.command_topic)

    # ------------------------------------------------------------------
    # Parameter and parsing helpers.
    def _load_bool(self, param_name, default):
        value = rospy.get_param(param_name, default)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _load_int_list(self, param_name, default):
        value = rospy.get_param(param_name, default)
        if isinstance(value, bool):
            return [int(value)]
        if isinstance(value, int):
            return [value]
        if isinstance(value, float):
            return [int(value)]
        if isinstance(value, str):
            return [int(v.strip()) for v in value.split(",") if v.strip()]
        return [int(v) for v in value]

    def _split_command(self, text):
        normalized = text.strip()
        for separator in (":", "=", ","):
            normalized = normalized.replace(separator, " ")
        return [part for part in normalized.split() if part]

    def _parse_int(self, value):
        return int(value, 0)

    def _read_value(self, args, command_name):
        if not args:
            rospy.logwarn("command %s requires an integer value", command_name)
            return None
        try:
            return self._parse_int(args[0])
        except ValueError:
            rospy.logwarn("invalid integer value for %s: %s", command_name, args[0])
            return None

    def _sleep_gap(self, duration=None):
        duration = self.command_gap_sec if duration is None else float(duration)
        if duration > 0.0:
            rospy.sleep(duration)

    # ------------------------------------------------------------------
    # Layer 1: ROS raw output. No state assumptions, no task policy.
    def raw_send_simple(self, code, value=0, cmd_type=Lite3Protocol.TYPE_SIMPLE):
        msg = SimpleCMD()
        msg.cmd_code = int(code)
        msg.cmd_value = int(value)
        msg.type = int(cmd_type)
        self.simple_pub.publish(msg)
        rospy.loginfo("sent simple_cmd code=0x%08X value=%d type=%d", code, value, cmd_type)

    def raw_send_zero_velocity(self, duration=None):
        duration = self.stop_duration_sec if duration is None else float(duration)
        zero = Twist()
        rate = rospy.Rate(self.stop_publish_hz)
        deadline = time.time() + max(0.0, duration)
        while not rospy.is_shutdown() and time.time() < deadline:
            self.cmd_vel_pub.publish(zero)
            rate.sleep()
        self.cmd_vel_pub.publish(zero)
        rospy.loginfo("sent zero /cmd_vel for %.2fs", duration)

    # ------------------------------------------------------------------
    # Layer 2: state feedback.
    def _on_robot_basic_state(self, msg):
        self._robot_basic_state = int(msg.data)
        self._robot_state_time = time.time()

    def _on_robot_gait_state(self, msg):
        self._robot_gait_state = int(msg.data)

    def _on_robot_motion_state(self, msg):
        self._robot_motion_state = int(msg.data)

    def state_text(self):
        return "basic=%s gait=%s motion=%s" % (
            self._robot_basic_state,
            self._robot_gait_state,
            self._robot_motion_state,
        )

    def has_fresh_robot_state(self):
        return (
            self._robot_basic_state is not None
            and (time.time() - self._robot_state_time) <= self.robot_state_timeout_sec
        )

    def is_in_basic_state(self, state_values):
        return self.has_fresh_robot_state() and self._robot_basic_state in state_values

    def is_lie_down(self):
        return self.is_in_basic_state(self.lie_basic_states)

    def is_standing(self):
        return self.is_in_basic_state(self.standing_basic_states)

    def wait_for_basic_state(self, state_values, label, timeout=None):
        timeout = self.mode_switch_timeout_sec if timeout is None else float(timeout)
        deadline = time.time() + max(0.0, timeout)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.is_in_basic_state(state_values):
                rospy.loginfo("Lite3 %s confirmed, %s", label, self.state_text())
                return True
            rospy.sleep(0.05)
        if self.has_fresh_robot_state():
            rospy.logwarn("Lite3 %s not confirmed after %.2fs, %s, expected=%s",
                          label, timeout, self.state_text(), state_values)
        else:
            rospy.logwarn("Lite3 %s not confirmed after %.2fs: no fresh robot state", label, timeout)
        return False

    # ------------------------------------------------------------------
    # Layer 3: guarded single-purpose operations.
    def send_mode_command(self, code, expected_states, label):
        for _ in range(max(1, self.mode_switch_retry_count)):
            self.raw_send_simple(code)
            if self.wait_for_basic_state(expected_states, label):
                return True
        return False

    def ensure_spot_mode(self):
        return self.send_mode_command(Lite3Protocol.SPOT_MODE, self.spot_basic_states, "spot/attitude mode")

    def ensure_move_mode(self):
        return self.send_mode_command(Lite3Protocol.MOVE_MODE, self.move_basic_states, "move mode")

    def ensure_standing(self):
        if self.is_standing():
            rospy.loginfo("Lite3 already standing, %s", self.state_text())
            if self.enter_move_mode_after_stand:
                self.ensure_move_mode()
            return True

        if self.has_fresh_robot_state():
            if not self.is_lie_down():
                rospy.logwarn("Lite3 is not in confirmed lie-down state before stand toggle, %s", self.state_text())
        else:
            rospy.logwarn("No fresh Lite3 state before stand toggle; sending toggle as fallback.")

        self.raw_send_simple(Lite3Protocol.STAND_LIE_TOGGLE)
        ok = self.wait_for_basic_state(self.standing_basic_states, "standing", self.stand_timeout_sec)
        self._sleep_gap(self.stand_settle_sec)
        if ok and self.enter_move_mode_after_stand:
            self.ensure_move_mode()
        return ok

    def lie_down_if_standing(self):
        self.raw_send_zero_velocity()
        if self.is_lie_down():
            rospy.loginfo("Lite3 already lying down, %s", self.state_text())
            return True
        if not self.is_standing():
            rospy.logwarn("Refuse lie-down toggle without confirmed standing state, %s", self.state_text())
            return False
        self.raw_send_simple(Lite3Protocol.STAND_LIE_TOGGLE)
        return self.wait_for_basic_state(self.lie_basic_states, "lie down", self.stand_timeout_sec)

    def command_auto_mode(self):
        self.raw_send_simple(Lite3Protocol.AUTO_MODE)

    def command_manual_mode(self):
        self.raw_send_simple(Lite3Protocol.MANUAL_MODE)

    def ensure_velocity_control_mode(self):
        self.command_auto_mode()
        self._sleep_gap()
        return self.ensure_move_mode()

    def command_zero_position(self):
        self.raw_send_simple(Lite3Protocol.ZERO_POSITION)

    def command_soft_estop(self):
        self.raw_send_simple(Lite3Protocol.SOFT_ESTOP)

    def command_axis_spot(self, code, value, label):
        self.ensure_spot_mode()
        rospy.loginfo("Lite3 spot axis %s value=%d", label, value)
        self.raw_send_simple(code, value)

    def command_axis_move(self, code, value, label):
        self.ensure_move_mode()
        rospy.loginfo("Lite3 move axis %s value=%d", label, value)
        self.raw_send_simple(code, value)

    def command_adjust_height(self, value):
        self.command_axis_spot(Lite3Protocol.AXIS_HEIGHT, value, "height")

    def command_adjust_roll(self, value):
        self.command_axis_spot(Lite3Protocol.AXIS_ROLL_OR_SIDE, value, "roll")

    def command_adjust_pitch(self, value):
        self.command_axis_spot(Lite3Protocol.AXIS_PITCH_OR_FORWARD, value, "pitch")

    def command_adjust_yaw(self, value):
        self.command_axis_spot(Lite3Protocol.AXIS_YAW_OR_TURN, value, "yaw")

    def command_move_forward_axis(self, value):
        self.command_axis_move(Lite3Protocol.AXIS_PITCH_OR_FORWARD, value, "forward")

    def command_move_side_axis(self, value):
        self.command_axis_move(Lite3Protocol.AXIS_ROLL_OR_SIDE, value, "side")

    def command_turn_axis(self, value):
        self.command_axis_move(Lite3Protocol.AXIS_YAW_OR_TURN, value, "turn")

    def command_gait(self, code, label):
        self.raw_send_simple(code)
        rospy.loginfo("Lite3 gait command: %s", label)

    def command_continuous_motion(self, enabled):
        value = Lite3Protocol.CONTINUOUS_MOTION_ON if enabled else Lite3Protocol.CONTINUOUS_MOTION_OFF
        self.raw_send_simple(Lite3Protocol.CONTINUOUS_MOTION, value)

    def command_speaker(self, value):
        self.raw_send_simple(Lite3Protocol.SPEAKER, value)

    def command_voice(self, value):
        self.raw_send_simple(Lite3Protocol.VOICE_COMMAND, value)

    def command_special_action(self, code):
        self.raw_send_simple(code)

    def command_ai_option(self, value):
        self.raw_send_simple(Lite3Protocol.AI_OPTION, value)

    # ------------------------------------------------------------------
    # Layer 4: task recipes used by competition flows.
    def prepare_navigation(self):
        if not self.ensure_standing():
            return False
        self.ensure_velocity_control_mode()
        self._sleep_gap()
        self.run_named_command(self.default_prepare_gait)
        self._sleep_gap()
        self.raw_send_zero_velocity(0.3)
        return True

    def prepare_hardcoded_motion(self):
        return self.prepare_navigation()

    def enter_inspection_view_pose(self):
        self.raw_send_zero_velocity()
        self.ensure_spot_mode()
        self._sleep_gap(self.view_pose_step_sleep_sec)
        for _ in range(max(1, self.view_pose_pitch_repeat_count)):
            self.raw_send_simple(Lite3Protocol.AXIS_PITCH_OR_FORWARD, self.inspection_pitch_value)
            self._sleep_gap(self.view_pose_step_sleep_sec)
        return True

    def restore_navigation_view_pose(self):
        self.ensure_spot_mode()
        for _ in range(max(1, self.view_pose_pitch_repeat_count)):
            self.raw_send_simple(Lite3Protocol.AXIS_PITCH_OR_FORWARD, self.navigation_pitch_value)
            self._sleep_gap(self.view_pose_step_sleep_sec)
        self.ensure_velocity_control_mode()
        return True

    def set_low_body_height(self):
        self.command_adjust_height(self.low_pose_height_value)

    def restore_body_height(self):
        self.command_adjust_height(self.normal_height_value)

    def toggle_crawl_gait(self):
        self.ensure_move_mode()
        self.raw_send_simple(Lite3Protocol.GAIT_CRAWL_TOGGLE)

    def command_raw_simple(self, args):
        if not args:
            rospy.logwarn("raw/simple requires at least cmd_code, e.g. raw:0x21010102:-20000:0")
            return
        try:
            code = self._parse_int(args[0])
            value = self._parse_int(args[1]) if len(args) > 1 else 0
            cmd_type = self._parse_int(args[2]) if len(args) > 2 else 0
        except ValueError as exc:
            rospy.logwarn("invalid raw/simple command args %s: %s", args, exc)
            return
        self.raw_send_simple(code, value, cmd_type)

    def command_send_value(self, args, command_name, callback):
        value = self._read_value(args, command_name)
        if value is None:
            return
        callback(value)

    def run_named_command(self, command_text):
        self._on_command(String(data=command_text))

    def _build_dispatch_table(self):
        self.command_handlers = {}

        def add(names, handler):
            for name in names:
                self.command_handlers[name] = handler

        add(("prepare_navigation", "prepare_nav", "nav_prepare"),
            lambda args, name: self.prepare_navigation())
        add(("prepare_hardcoded_motion", "prepare_hardcoded", "hardcoded_prepare"),
            lambda args, name: self.prepare_hardcoded_motion())

        add(("ensure_stand", "ensure_standing", "stand_if_needed"),
            lambda args, name: self.ensure_standing())
        add(("stand_lie_toggle_raw", "stand_toggle", "stand", "stand_up"),
            lambda args, name: self.raw_send_simple(Lite3Protocol.STAND_LIE_TOGGLE))
        add(("lie_down_if_standing", "lie", "lie_down", "sit_down"),
            lambda args, name: self.lie_down_if_standing())

        add(("move_mode", "move"), lambda args, name: self.ensure_move_mode())
        add(("spot_mode", "spot", "attitude_mode"), lambda args, name: self.ensure_spot_mode())
        add(("velocity_control_mode", "cmd_vel_mode", "nav_motion_mode"),
            lambda args, name: self.ensure_velocity_control_mode())
        add(("auto_mode", "autonomous"), lambda args, name: self.command_auto_mode())
        add(("manual_mode", "manual"), lambda args, name: self.command_manual_mode())
        add(("zero_position", "zero_pos"), lambda args, name: self.command_zero_position())
        add(("stop", "zero_vel", "zero_velocity"), lambda args, name: self.raw_send_zero_velocity())
        add(("estop", "soft_estop", "soft_emergency_stop"), lambda args, name: self.command_soft_estop())

        add(("inspection_view_pose",), lambda args, name: self.enter_inspection_view_pose())
        add(("navigation_view_pose",), lambda args, name: self.restore_navigation_view_pose())

        add(("height", "body_height", "adjust_height"),
            lambda args, name: self.command_send_value(args, name, self.command_adjust_height))
        add(("height_low", "low_height", "set_low_body_height"),
            lambda args, name: self.set_low_body_height())
        add(("height_normal", "normal_height", "restore_body_height", "normal_pose"),
            lambda args, name: self.restore_body_height())
        add(("crawl_gait_toggle", "toggle_crawl_gait", "crawl"),
            lambda args, name: self.toggle_crawl_gait())
        add(("low_pose", "crawl_pose", "prone_pose"),
            lambda args, name: (self.set_low_body_height(), self.toggle_crawl_gait()))

        add(("roll", "adjust_roll"),
            lambda args, name: self.command_send_value(args, name, self.command_adjust_roll))
        add(("pitch", "adjust_pitch"),
            lambda args, name: self.command_send_value(args, name, self.command_adjust_pitch))
        add(("yaw", "adjust_yaw"),
            lambda args, name: self.command_send_value(args, name, self.command_adjust_yaw))
        add(("forward_axis", "move_forward_axis", "forward", "move_forward"),
            lambda args, name: self.command_send_value(args, name, self.command_move_forward_axis))
        add(("side_axis", "strafe_axis", "side", "strafe", "move_side"),
            lambda args, name: self.command_send_value(args, name, self.command_move_side_axis))
        add(("turn_axis", "raw_turn", "turn"),
            lambda args, name: self.command_send_value(args, name, self.command_turn_axis))

        add(("flat_low_gait", "low_gait"), lambda args, name: self.command_gait(Lite3Protocol.GAIT_FLAT_LOW, name))
        add(("flat_medium_gait", "medium_gait"), lambda args, name: self.command_gait(Lite3Protocol.GAIT_FLAT_MEDIUM, name))
        add(("flat_high_gait", "high_gait"), lambda args, name: self.command_gait(Lite3Protocol.GAIT_FLAT_HIGH, name))
        add(("grip_obstacle_gait",), lambda args, name: self.command_gait(Lite3Protocol.GAIT_GRIP_OBSTACLE, name))
        add(("general_obstacle_gait",), lambda args, name: self.command_gait(Lite3Protocol.GAIT_GENERAL_OBSTACLE, name))
        add(("high_step_gait",), lambda args, name: self.command_gait(Lite3Protocol.GAIT_HIGH_STEP, name))

        add(("continuous_motion_on",), lambda args, name: self.command_continuous_motion(True))
        add(("continuous_motion_off",), lambda args, name: self.command_continuous_motion(False))
        add(("speaker_on",), lambda args, name: self.command_speaker(Lite3Protocol.SPEAKER_ON))
        add(("speaker_off",), lambda args, name: self.command_speaker(Lite3Protocol.SPEAKER_OFF))
        add(("speaker_query",), lambda args, name: self.command_speaker(Lite3Protocol.SPEAKER_QUERY))
        add(("voice", "voice_cmd"),
            lambda args, name: self.command_send_value(args, name, self.command_voice))

        add(("twist_body", "twist"), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_TWIST_BODY))
        add(("flip",), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_FLIP))
        add(("space_step", "space_walk"), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_SPACE_STEP))
        add(("backflip",), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_BACKFLIP))
        add(("greet",), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_GREET))
        add(("forward_jump",), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_FORWARD_JUMP))
        add(("twist_jump",), lambda args, name: self.command_special_action(Lite3Protocol.ACTION_TWIST_JUMP))

        add(("ai_off", "close_ai", "close_all_ai"), lambda args, name: self.command_ai_option(Lite3Protocol.AI_DISABLE_ALL))
        add(("obstacle_stop", "enable_obstacle_stop"),
            lambda args, name: self.command_ai_option(Lite3Protocol.AI_OBSTACLE_STOP))
        add(("follow", "enable_follow"), lambda args, name: self.command_ai_option(Lite3Protocol.AI_FOLLOW))
        add(("raw", "simple"), lambda args, name: self.command_raw_simple(args))

    def _on_command(self, msg):
        parts = self._split_command(msg.data)
        if not parts:
            return
        command = parts[0].lower()
        args = parts[1:]
        handler = self.command_handlers.get(command)
        if handler is None:
            rospy.logwarn("unknown lite3 motion command: %s", msg.data)
            return
        handler(args, command)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    Lite3MotionCmd().spin()
