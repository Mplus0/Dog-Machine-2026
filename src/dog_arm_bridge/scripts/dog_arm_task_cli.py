#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import threading
import time

import rospy
from std_msgs.msg import String


class DogArmTaskCli(object):
    def __init__(self, args):
        rospy.init_node("dog_arm_task_cli", anonymous=True)
        self.args = args
        self.result = None
        self.cv = threading.Condition()
        self.pub = rospy.Publisher(args.task_cmd_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(args.task_result_topic, String, self._on_result, queue_size=10)

    def _on_result(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("invalid arm result JSON: %s", msg.data)
            return
        if str(data.get("task_id", "")) != str(self.args.task_id):
            return
        with self.cv:
            self.result = data
            self.cv.notify_all()

    def wait_for_connection(self):
        deadline = time.time() + max(0.0, self.args.connect_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            if time.time() > deadline:
                return False
            rate.sleep()
        return True

    def run(self):
        if not self.wait_for_connection():
            raise RuntimeError("no subscriber on %s" % self.args.task_cmd_topic)

        payload = json.dumps(
            {
                "task_id": self.args.task_id,
                "cmd": self.args.cmd,
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        rospy.loginfo("publish %s: %s", self.args.task_cmd_topic, payload)
        self.pub.publish(String(data=payload))

        deadline = time.time() + max(0.0, self.args.timeout)
        with self.cv:
            while not rospy.is_shutdown():
                if self.result is not None:
                    print(json.dumps(self.result, ensure_ascii=False, sort_keys=True))
                    return 0 if self._is_success(self.result) else 2
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    print("timeout waiting for task_result task_id=%s" % self.args.task_id)
                    return 1
                self.cv.wait(min(0.2, remaining))
        return 1

    def _is_success(self, data):
        expected = "pick_success" if self.args.cmd == "pick" else "place_success"
        return data.get("result") == expected


def parse_args():
    parser = argparse.ArgumentParser(description="Send one dog-arm task command and wait for matching result.")
    parser.add_argument("cmd", choices=("pick", "place_to_zone"))
    parser.add_argument("--task-id", default="manual_001")
    parser.add_argument("--task-cmd-topic", default="/dog_arm/task_cmd")
    parser.add_argument("--task-result-topic", default="/dog_arm/task_result")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    return parser.parse_args()


def main():
    args = parse_args()
    code = DogArmTaskCli(args).run()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
