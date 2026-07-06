#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import datetime as _dt
import json
import os
import platform
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time


PROFILES = {
    "nav": [
        "/tf",
        "/tf_static",
        "/scan",
        "/map",
        "/map_metadata",
        "/amcl_map",
        "/amcl_map_metadata",
        "/amcl_pose",
        "/particlecloud",
        "/initialpose",
        "/move_base_simple/goal",
        "/move_base/status",
        "/move_base/feedback",
        "/move_base/result",
        "/move_base/goal",
        "/move_base/cancel",
        "/move_base/NavfnROS/plan",
        "/move_base/TebLocalPlannerROS/local_plan",
        "/move_base/global_costmap/costmap",
        "/move_base/global_costmap/costmap_updates",
        "/move_base/local_costmap/costmap",
        "/move_base/local_costmap/costmap_updates",
        "/cmd_vel",
        "/odom",
        "/leg_odom2",
        "/lite3/robot_basic_state",
        "/lite3/robot_gait_state",
        "/lite3/robot_motion_state",
        "/simple_cmd",
        "/lite3_motion_cmd",
        "/obstacle_task/report",
        "/obstacle_task/succeeded",
        "/inspect_report",
        "/full_task/report",
        "/pick_place/report",
    ],
    "perception": [
        "/tf",
        "/tf_static",
        "/camera/color/image_raw",
        "/camera/color/camera_info",
        "/camera/depth/image_rect_raw",
        "/camera/depth/camera_info",
        "/camera/aligned_depth_to_color/image_raw",
        "/camera/aligned_depth_to_color/camera_info",
        "/scan",
        "/meter_inspection_ready",
        "/meter_inspect_trigger",
        "/meter_status",
        "/meter_state_json",
        "/inspect_report",
    ],
    "full": [
        "/tf",
        "/tf_static",
        "/scan",
        "/map",
        "/map_metadata",
        "/amcl_map",
        "/amcl_map_metadata",
        "/amcl_pose",
        "/particlecloud",
        "/initialpose",
        "/move_base_simple/goal",
        "/move_base/status",
        "/move_base/feedback",
        "/move_base/result",
        "/move_base/goal",
        "/move_base/cancel",
        "/move_base/NavfnROS/plan",
        "/move_base/TebLocalPlannerROS/local_plan",
        "/move_base/global_costmap/costmap",
        "/move_base/global_costmap/costmap_updates",
        "/move_base/local_costmap/costmap",
        "/move_base/local_costmap/costmap_updates",
        "/camera/color/image_raw",
        "/camera/color/camera_info",
        "/camera/depth/image_rect_raw",
        "/camera/depth/camera_info",
        "/camera/aligned_depth_to_color/image_raw",
        "/camera/aligned_depth_to_color/camera_info",
        "/cmd_vel",
        "/odom",
        "/leg_odom2",
        "/lite3/robot_basic_state",
        "/lite3/robot_gait_state",
        "/lite3/robot_motion_state",
        "/simple_cmd",
        "/lite3_motion_cmd",
        "/meter_inspection_ready",
        "/meter_inspect_trigger",
        "/meter_status",
        "/meter_state_json",
        "/obstacle_task/report",
        "/obstacle_task/succeeded",
        "/inspect_report",
        "/full_task/report",
        "/pick_place/report",
    ],
    "state": [
        "/tf",
        "/tf_static",
        "/scan",
        "/amcl_pose",
        "/particlecloud",
        "/move_base/status",
        "/move_base/feedback",
        "/move_base/NavfnROS/plan",
        "/move_base/TebLocalPlannerROS/local_plan",
        "/cmd_vel",
        "/odom",
        "/leg_odom2",
        "/lite3/robot_basic_state",
        "/simple_cmd",
        "/lite3_motion_cmd",
        "/meter_inspection_ready",
        "/meter_inspect_trigger",
        "/meter_status",
        "/meter_state_json",
        "/obstacle_task/report",
        "/inspect_report",
        "/full_task/report",
    ],
}


def run_capture(command):
    try:
        return subprocess.check_output(command, stderr=subprocess.STDOUT, text=True, timeout=10)
    except (OSError, subprocess.CalledProcessError) as exc:
        return None
    except subprocess.TimeoutExpired:
        return None


def list_topics():
    output = run_capture(["rostopic", "list"])
    if output is None:
        return None
    return set(line.strip() for line in output.splitlines() if line.strip())


def topic_info(topic):
    output = run_capture(["rostopic", "info", topic])
    return output.strip() if output else ""


def measure_topic_hz(topic, duration_sec):
    command = ["rostopic", "hz", topic, "-w", "20"]
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            output, _ = process.communicate(timeout=max(1.0, duration_sec))
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                output, _ = process.communicate(timeout=2.0)
    except OSError as exc:
        return {"ok": False, "error": str(exc), "average_rate": None, "output": ""}

    average_rate = None
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("average rate:"):
            try:
                average_rate = float(line.split(":", 1)[1].strip())
            except ValueError:
                average_rate = None
    return {
        "ok": average_rate is not None,
        "average_rate": average_rate,
        "output": output.strip(),
    }


def collect_hz_report(topics, duration_sec):
    report = {}
    for topic in topics:
        report[topic] = measure_topic_hz(topic, duration_sec)
    return report


def unique_keep_order(items):
    seen = set()
    result = []
    for item in items:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result


def shell_join(args):
    return " ".join(shlex.quote(arg) for arg in args)


def disk_usage(path):
    try:
        usage = shutil.disk_usage(path)
    except OSError as exc:
        return {"error": str(exc)}
    return {
        "total_bytes": usage.total,
        "used_bytes": usage.used,
        "free_bytes": usage.free,
        "free_gb": round(usage.free / (1024.0 ** 3), 3),
    }


def command_version(command):
    output = run_capture(command)
    if output is None:
        return None
    return output.strip().splitlines()[:5]


def collect_environment(output_dir):
    env_keys = [
        "ROS_MASTER_URI",
        "ROS_HOSTNAME",
        "ROS_IP",
        "ROS_PACKAGE_PATH",
        "PYTHONPATH",
        "USER",
        "HOME",
    ]
    return {
        "hostname": socket.gethostname(),
        "cwd": os.getcwd(),
        "python": sys.version,
        "platform": platform.platform(),
        "env": {key: os.environ.get(key, "") for key in env_keys},
        "git": {
            "commit": (run_capture(["git", "rev-parse", "HEAD"]) or "").strip(),
            "branch": (run_capture(["git", "rev-parse", "--abbrev-ref", "HEAD"]) or "").strip(),
            "status_short": (run_capture(["git", "status", "--short"]) or "").splitlines(),
        },
        "disk": {
            "output_dir": disk_usage(output_dir),
            "home": disk_usage(os.path.expanduser("~")),
            "cwd": disk_usage(os.getcwd()),
        },
        "versions": {
            "rosbag": command_version(["rosbag", "--help"]),
            "rostopic": command_version(["rostopic", "--help"]),
            "docker": command_version(["docker", "--version"]),
        },
        "rosparam_list": (run_capture(["rosparam", "list"]) or "").splitlines(),
    }


def build_topics(args):
    topics = []
    for profile in args.profile:
        topics.extend(PROFILES[profile])
    topics.extend(args.topic)
    if args.no_rgb:
        topics = [t for t in topics if "/camera/color/" not in t and "aligned_depth_to_color" not in t]
    if args.no_depth:
        topics = [t for t in topics if "/camera/depth/" not in t and "aligned_depth_to_color" not in t]
    if args.no_costmap:
        topics = [t for t in topics if "costmap" not in t]
    return unique_keep_order(topics)


def build_output_prefix(args):
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "%s_%s" % (args.prefix, stamp))


def build_record_command(args, topics, output_prefix):
    command = ["rosbag", "record"]
    if args.quiet:
        command.append("--quiet")
    if args.tcpnodelay:
        command.append("--tcpnodelay")
    if args.bz2:
        command.append("--bz2")
    elif args.lz4:
        command.append("--lz4")
    if args.buffsize:
        command.append("--buffsize=%d" % args.buffsize)
    if args.chunksize:
        command.append("--chunksize=%d" % args.chunksize)
    if args.split:
        command.append("--split")
        if args.split_size:
            command.append("--size=%d" % args.split_size)
        if args.split_duration:
            command.append("--duration=%s" % args.split_duration)
        if args.max_splits:
            command.append("--max-splits=%d" % args.max_splits)
    command.extend(["-O", output_prefix + ".bag"])
    command.extend(topics)
    return command


def dump_rosparams(path):
    try:
        result = subprocess.run(
            ["rosparam", "dump", path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"ok": False, "error": str(exc), "path": path}
    return {
        "ok": result.returncode == 0,
        "returncode": result.returncode,
        "output": result.stdout.strip(),
        "path": path,
    }


def write_manifest(path, args, topics, found_topics, output_prefix, command, hz_report, rosparam_dump):
    output_dir = os.path.dirname(output_prefix)
    payload = {
        "created_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "profiles": args.profile,
        "output_prefix": output_prefix,
        "command": command,
        "topics": topics,
        "missing_topics": sorted(set(topics) - found_topics) if found_topics is not None else None,
        "present_topics": sorted(set(topics) & found_topics) if found_topics is not None else None,
        "topic_info": {},
        "hz_report": hz_report,
        "rosparam_dump": rosparam_dump,
        "environment": collect_environment(output_dir),
    }
    if found_topics is not None:
        for topic in topics:
            if topic in found_topics:
                payload["topic_info"][topic] = topic_info(topic)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")


def print_topic_report(topics, found_topics):
    if found_topics is None:
        print("warning: cannot run `rostopic list`; ROS master may not be available", file=sys.stderr)
        return
    present = [t for t in topics if t in found_topics]
    missing = [t for t in topics if t not in found_topics]
    print("topic check: present=%d missing=%d" % (len(present), len(missing)))
    if missing:
        print("missing topics:")
        for topic in missing:
            print("  " + topic)


def wait_for_any_topic(topics, timeout_sec):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        found = list_topics()
        if found is not None and any(topic in found for topic in topics):
            return found
        time.sleep(1.0)
    return list_topics()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record competition rosbag profiles for later offline replay."
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILES),
        action="append",
        default=None,
        help="Topic profile to record. Can be repeated. Default: full",
    )
    parser.add_argument("--topic", action="append", default=[], help="Extra topic to record. Can be repeated.")
    parser.add_argument("--output-dir", default="~/bags", help="Directory for bag and manifest.")
    parser.add_argument("--prefix", default="task", help="Output filename prefix.")
    parser.add_argument("--dry-run", action="store_true", help="Print command and exit.")
    parser.add_argument("--check-only", action="store_true", help="Only check topic availability and exit.")
    parser.add_argument("--wait", type=float, default=0.0, help="Wait seconds for at least one selected topic.")
    parser.add_argument("--hz-check", action="store_true", help="Measure key topic frequencies before recording.")
    parser.add_argument("--hz-duration", type=float, default=4.0, help="Seconds to sample each topic for --hz-check.")
    parser.add_argument(
        "--hz-topic",
        action="append",
        default=[],
        help="Extra topic for --hz-check. Can be repeated.",
    )
    parser.add_argument(
        "--rosparam-dump",
        action="store_true",
        help="Also write a rosparam dump next to the bag manifest.",
    )
    parser.add_argument("--no-rgb", action="store_true", help="Drop RGB image topics from selected profiles.")
    parser.add_argument("--no-depth", action="store_true", help="Drop depth image topics from selected profiles.")
    parser.add_argument("--no-costmap", action="store_true", help="Drop costmap topics from selected profiles.")
    parser.add_argument("--split", action="store_true", help="Enable rosbag split mode.")
    parser.add_argument("--split-size", type=int, default=4096, help="Split bag size in MB when --split is used.")
    parser.add_argument("--split-duration", default="", help="Split duration passed to rosbag, for example 300.")
    parser.add_argument("--max-splits", type=int, default=0, help="Maximum number of split bag files.")
    parser.add_argument("--buffsize", type=int, default=2048, help="rosbag buffer size in MB.")
    parser.add_argument("--chunksize", type=int, default=768, help="rosbag chunk size in KB.")
    parser.add_argument("--lz4", action="store_true", default=True, help="Use lz4 compression. Default on.")
    parser.add_argument("--no-lz4", action="store_false", dest="lz4", help="Disable lz4 compression.")
    parser.add_argument("--bz2", action="store_true", help="Use bz2 compression instead of lz4.")
    parser.add_argument("--tcpnodelay", action="store_true", help="Pass --tcpnodelay to rosbag record.")
    parser.add_argument("--quiet", action="store_true", help="Pass --quiet to rosbag record.")
    args = parser.parse_args()
    if args.profile is None:
        args.profile = ["full"]
    return args


def main():
    args = parse_args()
    topics = build_topics(args)
    output_prefix = build_output_prefix(args)

    found_topics = wait_for_any_topic(topics, args.wait) if args.wait > 0.0 else list_topics()
    print_topic_report(topics, found_topics)

    manifest_path = output_prefix + "_manifest.json"
    command = build_record_command(args, topics, output_prefix)
    hz_report = {}
    if args.hz_check:
        hz_topics = unique_keep_order(
            [
                "/camera/depth/image_rect_raw",
                "/camera/color/image_raw",
                "/scan",
                "/amcl_pose",
                "/cmd_vel",
                "/move_base/status",
            ]
            + args.hz_topic
        )
        if found_topics is not None:
            hz_topics = [topic for topic in hz_topics if topic in found_topics]
        print("measuring topic hz: %s" % ", ".join(hz_topics))
        hz_report = collect_hz_report(hz_topics, args.hz_duration)

    print("command:")
    print("  " + shell_join(command))

    if args.dry_run:
        return 0

    rosparam_dump = None
    if args.rosparam_dump:
        rosparam_dump = dump_rosparams(output_prefix + "_rosparam.yaml")

    write_manifest(
        manifest_path,
        args,
        topics,
        found_topics,
        output_prefix,
        command,
        hz_report,
        rosparam_dump,
    )

    print("manifest: %s" % manifest_path)

    if args.check_only:
        return 0

    print("recording... press Ctrl-C to stop")
    process = subprocess.Popen(command)

    def stop(_signum, _frame):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    return process.wait()


if __name__ == "__main__":
    sys.exit(main())
