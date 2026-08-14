#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Record and classify AMCL/odometry jumps without commanding robot motion."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


BAG_TOPICS = [
    "/scan",
    "/scan_ground_filtered",
    "/scan_ground_clearing",
    "/imu/data_throttled",
    "/ground_filter/diagnostics",
    "/amcl_pose",
    "/particlecloud",
    "/odometry/filtered",
    "/amcl_map",
    "/amcl_map_metadata",
    "/map",
    "/map_metadata",
    "/tf",
    "/tf_static",
]


def clean_frame(frame):
    return frame.lstrip("/")


def quaternion_yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def angle_delta(current, previous):
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def finite_scan_summary(message):
    values = [
        value
        for value in message.ranges
        if math.isfinite(value) and message.range_min <= value <= message.range_max
    ]
    return {
        "stamp": message.header.stamp.to_sec(),
        "total": len(message.ranges),
        "finite": len(values),
        "nan": sum(math.isnan(value) for value in message.ranges),
        "inf": sum(math.isinf(value) for value in message.ranges),
        "min": min(values) if values else None,
        "median": statistics.median(values) if values else None,
    }


class JumpMonitor:
    def __init__(self, output_path, translation_threshold, yaw_threshold_deg):
        self.output_path = Path(output_path)
        self.jump_path = self.output_path.with_name("jumps.jsonl")
        self.output = self.output_path.open("a", encoding="utf-8", buffering=1)
        self.jumps = self.jump_path.open("a", encoding="utf-8", buffering=1)
        self.translation_threshold = translation_threshold
        self.yaw_threshold = math.radians(yaw_threshold_deg)
        self.previous = {}
        self.last_logged = {}
        self.latest_scan = {}
        self.latest_amcl = None
        self.latest_diagnostics = None
        self.recent_odom_jump_time = None

        self.write(
            "monitor_start",
            translation_threshold_m=translation_threshold,
            yaw_threshold_deg=yaw_threshold_deg,
        )
        rospy.Subscriber("/tf", TFMessage, self.tf_callback, queue_size=100)
        rospy.Subscriber(
            "/scan_ground_filtered", LaserScan, self.filtered_scan_callback, queue_size=2
        )
        rospy.Subscriber(
            "/scan_ground_clearing", LaserScan, self.clearing_scan_callback, queue_size=2
        )
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback, queue_size=10)
        rospy.Subscriber("/odometry/filtered", Odometry, self.odometry_callback, queue_size=20)
        rospy.Subscriber(
            "/ground_filter/diagnostics", DiagnosticArray, self.diagnostics_callback, queue_size=5
        )
        rospy.on_shutdown(self.close)

    def write(self, event_type, **values):
        record = {"event": event_type, "wall_time": time.time()}
        record.update(values)
        self.output.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")

    def write_jump(self, record):
        self.jumps.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self.output.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")

    def close(self):
        if not self.output.closed:
            self.write("monitor_stop")
            self.output.close()
        if not self.jumps.closed:
            self.jumps.close()

    def filtered_scan_callback(self, message):
        self.latest_scan["filtered"] = finite_scan_summary(message)

    def clearing_scan_callback(self, message):
        self.latest_scan["clearing"] = finite_scan_summary(message)

    def amcl_callback(self, message):
        covariance = message.pose.covariance
        pose = message.pose.pose
        self.latest_amcl = {
            "stamp": message.header.stamp.to_sec(),
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw_deg": math.degrees(quaternion_yaw(pose.orientation)),
            "cov_x": covariance[0],
            "cov_y": covariance[7],
            "cov_yaw": covariance[35],
        }
        self.write("amcl_pose", **self.latest_amcl)

    def odometry_callback(self, message):
        pose = message.pose.pose
        now = message.header.stamp.to_sec()
        if now - self.last_logged.get("odometry", -1e9) >= 0.2:
            self.last_logged["odometry"] = now
            self.write(
                "odometry",
                stamp=now,
                x=pose.position.x,
                y=pose.position.y,
                yaw_deg=math.degrees(quaternion_yaw(pose.orientation)),
            )

    def diagnostics_callback(self, message):
        for status in message.status:
            if status.name.endswith("/ground_filter"):
                self.latest_diagnostics = {
                    "stamp": message.header.stamp.to_sec(),
                    "level": int(status.level),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }
                break

    def tf_callback(self, message):
        for transform in message.transforms:
            parent = clean_frame(transform.header.frame_id)
            child = clean_frame(transform.child_frame_id)
            if (parent, child) == ("map", "odom"):
                self.process_transform("map_to_odom", transform)
            elif (parent, child) == ("odom", "base_link"):
                self.process_transform("odom_to_base_link", transform)

    def process_transform(self, name, transform):
        stamp = transform.header.stamp.to_sec()
        translation = transform.transform.translation
        yaw = quaternion_yaw(transform.transform.rotation)
        current = (translation.x, translation.y, yaw, stamp)
        previous = self.previous.get(name)
        self.previous[name] = current
        if previous is None:
            self.log_transform(name, current, 0.0, 0.0)
            return

        distance = math.hypot(current[0] - previous[0], current[1] - previous[1])
        yaw_change = abs(angle_delta(current[2], previous[2]))
        is_jump = distance > self.translation_threshold or yaw_change > self.yaw_threshold
        log_period = 0.2 if name == "odom_to_base_link" else 0.0
        if is_jump or stamp - self.last_logged.get(name, -1e9) >= log_period:
            self.log_transform(name, current, distance, yaw_change)

        if not is_jump:
            return
        if name == "odom_to_base_link":
            self.recent_odom_jump_time = stamp
        odom_nearby = (
            self.recent_odom_jump_time is not None
            and abs(stamp - self.recent_odom_jump_time) <= 0.5
        )
        classification = "ODOM_OR_SHARED" if odom_nearby else "AMCL_CANDIDATE"
        record = {
            "event": "jump",
            "wall_time": time.time(),
            "stamp": stamp,
            "transform": name,
            "translation_jump_m": distance,
            "yaw_jump_deg": math.degrees(yaw_change),
            "classification": classification,
            "current": {
                "x": current[0],
                "y": current[1],
                "yaw_deg": math.degrees(current[2]),
            },
            "amcl": self.latest_amcl,
            "scan": self.latest_scan,
            "ground_filter": self.latest_diagnostics,
        }
        self.write_jump(record)
        rospy.logwarn(
            "Jump detected on %s: %.3f m, %.2f deg (%s)",
            name,
            distance,
            math.degrees(yaw_change),
            classification,
        )

    def log_transform(self, name, current, distance, yaw_change):
        self.last_logged[name] = current[3]
        self.write(
            name,
            stamp=current[3],
            x=current[0],
            y=current[1],
            yaw_deg=math.degrees(current[2]),
            delta_translation_m=distance,
            delta_yaw_deg=math.degrees(yaw_change),
        )


def process_alive(pid):
    if not isinstance(pid, int) or pid <= 1:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def command_line(pid):
    try:
        return Path("/proc/{}/cmdline".format(pid)).read_bytes().replace(b"\0", b" ").decode()
    except (OSError, UnicodeDecodeError):
        return ""


def state_root(args):
    configured = args.output_root or os.environ.get("AMCL_JUMP_DIAG_ROOT")
    return Path(configured).expanduser() if configured else Path.home() / "amcl_jump_diagnostics"


def state_path(root):
    return root / "active.json"


def read_state(root):
    path = state_path(root)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def write_state(root, state):
    root.mkdir(parents=True, exist_ok=True)
    state_path(root).write_text(json.dumps(state, indent=2, sort_keys=True), encoding="utf-8")


def safe_label(label):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", label.strip())
    return cleaned.strip("_.") or "run"


def assert_ros_master():
    result = subprocess.run(
        ["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if result.returncode != 0:
        raise RuntimeError("ROS master is unavailable; start navigation before recording")


def capture_runtime_snapshot(run_dir):
    commands = [
        ("git_head", ["git", "-C", str(Path(__file__).resolve().parents[3]), "rev-parse", "HEAD"]),
        ("nodes", ["rosnode", "list"]),
        ("amcl_node", ["rosnode", "info", "/amcl"]),
        ("amcl_params", ["rosparam", "get", "/amcl"]),
        ("filtered_topic", ["rostopic", "info", "/scan_ground_filtered"]),
        ("clearing_topic", ["rostopic", "info", "/scan_ground_clearing"]),
        ("amcl_map_topic", ["rostopic", "info", "/amcl_map"]),
    ]
    snapshot_path = run_dir / "runtime_snapshot.txt"
    with snapshot_path.open("w", encoding="utf-8") as output:
        for title, command in commands:
            output.write("===== {} =====\n".format(title))
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=10.0,
            )
            output.write(result.stdout)
            output.write("exit_code={}\n\n".format(result.returncode))


def start_recording(args):
    root = state_root(args)
    old = read_state(root)
    if old and any(process_alive(old.get(key)) for key in ("monitor_pid", "rosbag_pid")):
        raise RuntimeError("a diagnostic run is already active: {}".format(old.get("run_dir")))
    assert_ros_master()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = root / "{}_{}".format(stamp, safe_label(args.label))
    run_dir.mkdir(parents=True, exist_ok=False)
    capture_runtime_snapshot(run_dir)
    monitor_log = (run_dir / "monitor_console.log").open("ab", buffering=0)
    rosbag_log = (run_dir / "rosbag_console.log").open("ab", buffering=0)
    script = os.path.realpath(__file__)
    monitor_command = [
        sys.executable,
        script,
        "_monitor",
        "--events",
        str(run_dir / "events.jsonl"),
        "--translation-threshold",
        str(args.translation_threshold),
        "--yaw-threshold-deg",
        str(args.yaw_threshold_deg),
    ]
    monitor = subprocess.Popen(
        monitor_command,
        stdout=monitor_log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rosbag_command = ["rosbag", "record", "-O", str(run_dir / "amcl_jump_diag.bag")]
    rosbag_command.extend(BAG_TOPICS)
    try:
        rosbag = subprocess.Popen(
            rosbag_command,
            stdout=rosbag_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except Exception:
        os.killpg(monitor.pid, signal.SIGINT)
        raise

    state = {
        "status": "running",
        "label": args.label,
        "started_at": time.time(),
        "run_dir": str(run_dir),
        "monitor_pid": monitor.pid,
        "rosbag_pid": rosbag.pid,
    }
    write_state(root, state)
    time.sleep(1.0)
    if not process_alive(monitor.pid) or not process_alive(rosbag.pid):
        raise RuntimeError("diagnostic process exited during startup; inspect logs in {}".format(run_dir))
    print("started: {}".format(run_dir))
    print("monitor_pid={} rosbag_pid={}".format(monitor.pid, rosbag.pid))
    print("This tool records only; it does not publish cmd_vel or clear/reset localization.")


def stop_process(pid, expected):
    if not process_alive(pid):
        return "already stopped"
    actual = command_line(pid)
    if expected not in actual:
        return "refused: PID command does not contain {!r}: {}".format(expected, actual)
    os.killpg(pid, signal.SIGINT)
    deadline = time.time() + 10.0
    while time.time() < deadline and process_alive(pid):
        time.sleep(0.2)
    if process_alive(pid):
        os.killpg(pid, signal.SIGTERM)
        return "SIGTERM after graceful timeout"
    return "stopped"


def stop_recording(args):
    root = state_root(args)
    state = read_state(root)
    if not state:
        print("no diagnostic state found")
        return
    print("rosbag: {}".format(stop_process(state.get("rosbag_pid"), "rosbag record")))
    print("monitor: {}".format(stop_process(state.get("monitor_pid"), "_monitor")))
    state["status"] = "stopped"
    state["stopped_at"] = time.time()
    write_state(root, state)
    print("saved: {}".format(state.get("run_dir")))


def print_status(args):
    root = state_root(args)
    state = read_state(root)
    if not state:
        print("status: idle (no state in {})".format(root))
        return
    print("run_dir: {}".format(state.get("run_dir")))
    print("state: {}".format(state.get("status")))
    for key in ("monitor_pid", "rosbag_pid"):
        pid = state.get(key)
        print("{}: {} alive={}".format(key, pid, process_alive(pid)))


def print_logs(args):
    root = state_root(args)
    state = read_state(root)
    if not state:
        print("no diagnostic state found")
        return
    run_dir = Path(state["run_dir"])
    print("run_dir: {}".format(run_dir))
    for name in ("jumps.jsonl", "monitor_console.log", "rosbag_console.log"):
        path = run_dir / name
        print("\n===== {} =====".format(path))
        if not path.exists():
            print("not created yet")
            continue
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        for line in lines[-args.lines :]:
            print(line)


def run_monitor(args):
    rospy.init_node("amcl_jump_monitor", anonymous=False)
    JumpMonitor(args.events, args.translation_threshold, args.yaw_threshold_deg)
    rospy.loginfo("AMCL jump monitor recording to %s", args.events)
    rospy.spin()


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "command", choices=("start", "stop", "status", "logs", "_monitor")
    )
    result.add_argument("label", nargs="?", default="run")
    result.add_argument("--output-root")
    result.add_argument("--events")
    result.add_argument("--translation-threshold", type=float, default=0.05)
    result.add_argument("--yaw-threshold-deg", type=float, default=2.0)
    result.add_argument("--lines", type=int, default=30)
    return result


def main():
    args = parser().parse_args()
    try:
        if args.command == "start":
            start_recording(args)
        elif args.command == "stop":
            stop_recording(args)
        elif args.command == "status":
            print_status(args)
        elif args.command == "logs":
            print_logs(args)
        elif args.command == "_monitor":
            if not args.events:
                raise RuntimeError("--events is required for internal monitor mode")
            run_monitor(args)
    except (OSError, RuntimeError) as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
