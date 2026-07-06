#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import time


DEFAULT_TOPICS = [
    "/tf",
    "/tf_static",
    "/camera/depth/image_rect_raw",
    "/camera/depth/camera_info",
    "/scan",
    "/map",
    "/amcl_pose",
    "/move_base/status",
    "/cmd_vel",
    "/leg_odom2",
    "/lite3_motion_cmd",
    "/meter_inspection_ready",
    "/meter_status",
]

DEFAULT_HZ_TOPICS = [
    "/camera/depth/image_rect_raw",
    "/scan",
    "/amcl_pose",
    "/move_base/status",
]


def run_capture(command, timeout=8):
    try:
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, str(exc), -1
    return result.stdout.strip(), "", result.returncode


def status(ok, detail="", warn=False):
    if ok:
        return {"level": "PASS", "detail": detail}
    return {"level": "WARN" if warn else "FAIL", "detail": detail}


def command_exists(name):
    return shutil.which(name) is not None


def list_topics():
    output, error, code = run_capture(["rostopic", "list"], timeout=8)
    if code != 0 or output is None:
        return None, output or error
    return set(line.strip() for line in output.splitlines() if line.strip()), ""


def list_nodes():
    output, error, code = run_capture(["rosnode", "list"], timeout=8)
    if code != 0 or output is None:
        return None, output or error
    return set(line.strip() for line in output.splitlines() if line.strip()), ""


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
        return {"ok": False, "average_rate": None, "detail": str(exc)}

    average_rate = None
    for line in output.splitlines():
        match = re.search(r"average rate:\s*([0-9.]+)", line)
        if match:
            average_rate = float(match.group(1))
    return {
        "ok": average_rate is not None,
        "average_rate": average_rate,
        "detail": output.strip(),
    }


def disk_report(path):
    path = os.path.abspath(os.path.expanduser(path))
    try:
        os.makedirs(path, exist_ok=True)
        usage = shutil.disk_usage(path)
    except OSError as exc:
        return {"ok": False, "path": path, "detail": str(exc)}
    return {
        "ok": True,
        "path": path,
        "free_gb": round(usage.free / (1024.0 ** 3), 2),
        "total_gb": round(usage.total / (1024.0 ** 3), 2),
    }


def ping_host(host):
    if platform.system().lower().startswith("win"):
        command = ["ping", "-n", "1", host]
    else:
        command = ["ping", "-c", "1", "-W", "1", host]
    output, error, code = run_capture(command, timeout=4)
    return code == 0, output or error


def check_file(path):
    path = os.path.abspath(os.path.expanduser(path))
    return os.path.exists(path), path


def docker_check():
    if not command_exists("docker"):
        return status(False, "docker command not found", warn=True)
    output, error, code = run_capture(["docker", "ps"], timeout=8)
    if code == 0:
        return status(True, "docker ps ok")
    return status(False, output or error, warn=True)


def git_snapshot():
    commit, _, _ = run_capture(["git", "rev-parse", "HEAD"], timeout=5)
    branch, _, _ = run_capture(["git", "rev-parse", "--abbrev-ref", "HEAD"], timeout=5)
    dirty, _, _ = run_capture(["git", "status", "--short"], timeout=5)
    return {
        "commit": (commit or "").strip(),
        "branch": (branch or "").strip(),
        "status_short": (dirty or "").splitlines(),
    }


def print_result(name, result):
    level = result.get("level", "INFO")
    detail = result.get("detail", "")
    print("[%s] %s%s" % (level, name, (" - " + detail) if detail else ""))


def parse_args():
    parser = argparse.ArgumentParser(description="Read-only preflight checks for robot tests.")
    parser.add_argument("--motion-host", default="192.168.1.120")
    parser.add_argument("--bags-dir", default="~/bags")
    parser.add_argument(
        "--model",
        default="~/comp2026_ws/src/dog_motion/models/yuyin.engine",
        help="Host-side TensorRT engine path.",
    )
    parser.add_argument("--min-free-gb", type=float, default=20.0)
    parser.add_argument("--topic", action="append", default=[], help="Extra required topic.")
    parser.add_argument("--skip-hz", action="store_true")
    parser.add_argument("--hz-topic", action="append", default=[], help="Extra topic for hz check.")
    parser.add_argument("--hz-duration", type=float, default=4.0)
    parser.add_argument("--json-output", default="", help="Optional JSON report path.")
    parser.add_argument("--strict", action="store_true", help="Return non-zero on FAIL.")
    return parser.parse_args()


def main():
    args = parse_args()
    checks = {}

    for command in ["rostopic", "rosnode", "rosbag", "roslaunch"]:
        checks["command:" + command] = status(command_exists(command), "%s in PATH" % command)
    checks["command:docker"] = docker_check()

    ok, detail = ping_host(args.motion_host)
    checks["motion_host_ping"] = status(ok, detail.splitlines()[-1] if detail else args.motion_host, warn=True)

    model_ok, model_path = check_file(args.model)
    checks["model_file"] = status(model_ok, model_path, warn=True)

    disk = disk_report(args.bags_dir)
    if disk["ok"]:
        checks["bags_disk"] = status(
            disk["free_gb"] >= args.min_free_gb,
            "%s free=%.2fGB total=%.2fGB" % (disk["path"], disk["free_gb"], disk["total_gb"]),
            warn=True,
        )
    else:
        checks["bags_disk"] = status(False, disk["detail"], warn=True)

    topics, topic_error = list_topics()
    required_topics = DEFAULT_TOPICS + args.topic
    topic_results = {}
    if topics is None:
        checks["ros_master_topics"] = status(False, topic_error)
    else:
        checks["ros_master_topics"] = status(True, "%d topics visible" % len(topics))
        for topic in required_topics:
            topic_results[topic] = topic in topics

    nodes, node_error = list_nodes()
    if nodes is None:
        checks["ros_nodes"] = status(False, node_error, warn=True)
    else:
        checks["ros_nodes"] = status(True, "%d nodes visible" % len(nodes))

    hz_report = {}
    if not args.skip_hz and topics is not None:
        hz_topics = [t for t in DEFAULT_HZ_TOPICS + args.hz_topic if t in topics]
        for topic in hz_topics:
            hz_report[topic] = measure_topic_hz(topic, args.hz_duration)

    report = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "hostname": socket.gethostname(),
        "cwd": os.getcwd(),
        "env": {
            "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI", ""),
            "ROS_IP": os.environ.get("ROS_IP", ""),
            "ROS_HOSTNAME": os.environ.get("ROS_HOSTNAME", ""),
        },
        "git": git_snapshot(),
        "checks": checks,
        "topics": {
            "required": required_topics,
            "present": sorted(topics) if topics is not None else None,
            "required_status": topic_results,
        },
        "hz_report": hz_report,
    }

    print("preflight")
    for name in sorted(checks):
        print_result(name, checks[name])
    if topic_results:
        missing = [topic for topic, ok in topic_results.items() if not ok]
        print("[PASS] required topics present=%d missing=%d" % (len(topic_results) - len(missing), len(missing)))
        for topic in missing:
            print("[WARN] missing topic - " + topic)
    for topic, item in hz_report.items():
        if item["ok"]:
            print("[PASS] hz %s - %.3f Hz" % (topic, item["average_rate"]))
        else:
            print("[WARN] hz %s - no average rate" % topic)

    if args.json_output:
        out_path = os.path.abspath(os.path.expanduser(args.json_output))
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
            f.write("\n")
        print("json report: " + out_path)

    has_fail = any(item.get("level") == "FAIL" for item in checks.values())
    return 1 if args.strict and has_fail else 0


if __name__ == "__main__":
    sys.exit(main())
