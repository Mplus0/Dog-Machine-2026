#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import threading

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


DEFAULT_OUTPUT = os.path.expanduser(
    "~/comp2026_ws/src/allmovebase/config/task_poses.yaml"
)


class LatestPose:
    def __init__(self):
        self._lock = threading.Lock()
        self.pose = None
        self.stamp = None

    def update_from_amcl(self, msg):
        with self._lock:
            self.pose = msg.pose.pose
            self.stamp = msg.header.stamp

    def update_from_pose_stamped(self, msg):
        with self._lock:
            self.pose = msg.pose
            self.stamp = msg.header.stamp

    def snapshot(self):
        with self._lock:
            return self.pose, self.stamp


def load_yaml(path):
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def save_yaml(path, data):
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)


def pose_to_data(pose, digits):
    return {
        "position": [
            round(float(pose.position.x), digits),
            round(float(pose.position.y), digits),
            round(float(pose.position.z), digits),
        ],
        "orientation": [
            round(float(pose.orientation.x), digits),
            round(float(pose.orientation.y), digits),
            round(float(pose.orientation.z), digits),
            round(float(pose.orientation.w), digits),
        ],
    }


def write_pose(path, namespace, name, pose, digits):
    data = load_yaml(path)
    if not isinstance(data, dict):
        data = {}
    section = data.setdefault(namespace, {})
    if not isinstance(section, dict):
        raise RuntimeError("YAML section %s is not a mapping" % namespace)
    section[name] = pose_to_data(pose, digits)
    save_yaml(path, data)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record the current robot pose into task_poses.yaml."
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help="YAML file to update. Defaults to ~/comp2026_ws/src/allmovebase/config/task_poses.yaml.",
    )
    parser.add_argument(
        "--pose-topic",
        default="/amcl_pose",
        help="Pose topic to listen to.",
    )
    parser.add_argument(
        "--pose-type",
        choices=("amcl", "pose_stamped"),
        default="amcl",
        help="Message type: amcl=PoseWithCovarianceStamped, pose_stamped=PoseStamped.",
    )
    parser.add_argument(
        "--namespace",
        choices=("goals", "waypoints"),
        default="goals",
        help="YAML section to update.",
    )
    parser.add_argument(
        "--digits",
        type=int,
        default=3,
        help="Decimal digits kept in YAML.",
    )
    parser.add_argument(
        "--name",
        default="",
        help="Record one pose with this name and exit. If omitted, interactive mode is used.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def subscribe_pose(args, latest_pose):
    if args.pose_type == "pose_stamped":
        return rospy.Subscriber(
            args.pose_topic,
            PoseStamped,
            latest_pose.update_from_pose_stamped,
            queue_size=1,
        )
    return rospy.Subscriber(
        args.pose_topic,
        PoseWithCovarianceStamped,
        latest_pose.update_from_amcl,
        queue_size=1,
    )


def wait_for_pose(latest_pose, timeout=10.0):
    rate = rospy.Rate(20)
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        pose, stamp = latest_pose.snapshot()
        if pose is not None:
            return pose, stamp
        rate.sleep()
    return None, None


def record_one(args, latest_pose, name, namespace):
    pose, stamp = wait_for_pose(latest_pose)
    if pose is None:
        rospy.logwarn("no pose received on %s", args.pose_topic)
        return False
    write_pose(args.output, namespace, name, pose, args.digits)
    stamp_text = "%.3f" % stamp.to_sec() if stamp is not None else "unknown"
    rospy.loginfo(
        "saved %s/%s from %s stamp=%s to %s",
        namespace,
        name,
        args.pose_topic,
        stamp_text,
        args.output,
    )
    return True


def interactive_loop(args, latest_pose):
    rospy.loginfo("recording pose topic: %s", args.pose_topic)
    rospy.loginfo("output YAML: %s", args.output)
    rospy.loginfo("default YAML section: %s", args.namespace)
    rospy.loginfo("input examples: rec_pose_1, waypoints obs_start, goals pickup_pose")
    rospy.loginfo("input q to quit")

    while not rospy.is_shutdown():
        try:
            text = input("pose name> ").strip()
        except EOFError:
            break
        if not text:
            continue
        if text.lower() in ("q", "quit", "exit"):
            break

        parts = text.split()
        namespace = args.namespace
        name = text
        if len(parts) == 2 and parts[0] in ("goals", "waypoints"):
            namespace, name = parts
        elif len(parts) != 1:
            rospy.logwarn("invalid input. Use: name OR goals name OR waypoints name")
            continue

        record_one(args, latest_pose, name, namespace)


def main():
    args = parse_args()
    rospy.init_node("record_task_pose", anonymous=False)

    latest_pose = LatestPose()
    subscribe_pose(args, latest_pose)

    if args.name:
        ok = record_one(args, latest_pose, args.name, args.namespace)
        if not ok:
            raise SystemExit(1)
        return

    interactive_loop(args, latest_pose)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
