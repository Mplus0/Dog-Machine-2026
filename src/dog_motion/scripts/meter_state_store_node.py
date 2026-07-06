#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import time

import rospy
import yaml
from std_msgs.msg import String


class MeterStateStoreNode:
    def __init__(self):
        rospy.init_node("meter_state_store_node", anonymous=True)

        self.status_topic = rospy.get_param("~status_topic", "/meter_status")
        self.state_topic = rospy.get_param("~state_topic", "/meter_state_json")
        self.param_name = rospy.get_param("~param_name", "/meter_states")
        self.ready_param_name = rospy.get_param("~ready_param_name", "/meter_states_ready")
        self.state_file = rospy.get_param(
            "~state_file", os.path.join(os.path.expanduser("~"), "meter_state.yaml")
        )
        self.clear_on_start = self._param_bool("~clear_on_start", True)
        self.expected_regions = self._parse_regions(
            rospy.get_param("~expected_regions", "A,B,C,D")
        )

        self.states = {}
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        rospy.Subscriber(self.status_topic, String, self._status_callback, queue_size=10)

        if self.clear_on_start:
            rospy.loginfo("clear existing meter states on start: file=%s", self.state_file)
        else:
            self._load_existing_state()
        self._publish_state()
        rospy.loginfo(
            "meter_state_store_node ready: status=%s param=%s file=%s clear_on_start=%s",
            self.status_topic,
            self.param_name,
            self.state_file,
            self.clear_on_start,
        )

    def _param_bool(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _parse_regions(self, value):
        if isinstance(value, str):
            return [item.strip().upper() for item in value.split(",") if item.strip()]
        return [str(item).strip().upper() for item in value if str(item).strip()]

    def _load_existing_state(self):
        if not os.path.exists(self.state_file):
            return
        try:
            with open(self.state_file, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            states = data.get("states", data)
            if isinstance(states, dict):
                for region, status in states.items():
                    region = str(region).strip().upper()
                    if region in self.expected_regions:
                        self.states[region] = str(status).strip().lower()
        except Exception as exc:
            rospy.logwarn("failed to load meter state file %s: %s", self.state_file, exc)

    def _status_callback(self, msg):
        data = str(msg.data).strip()
        parts = [item.strip() for item in data.split(",")]
        if len(parts) == 2:
            region, status = parts
        elif len(parts) == 3:
            _, region, status = parts
        else:
            rospy.logwarn("invalid meter status, expected 'A,normal' or 'rec_pose_1,A,normal': %s", data)
            return

        region = region.upper()
        status = status.lower()
        if region not in self.expected_regions:
            rospy.logwarn("unknown meter region %s from status %s", region, data)
            return
        if status not in ("low", "normal", "high"):
            rospy.logwarn("unknown meter status %s from status %s", status, data)
            return

        self.states[region] = status
        self._publish_state()
        rospy.loginfo("stored meter state: %s=%s", region, status)

    def _state_payload(self):
        return {
            "stamp": time.time(),
            "states": dict(sorted(self.states.items())),
            "complete": all(region in self.states for region in self.expected_regions),
            "expected_regions": list(self.expected_regions),
        }

    def _publish_state(self):
        payload = self._state_payload()
        rospy.set_param(self.param_name, payload["states"])
        rospy.set_param(self.ready_param_name, payload["complete"])
        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self._write_file(payload)

    def _write_file(self, payload):
        try:
            state_dir = os.path.dirname(self.state_file)
            if state_dir and not os.path.exists(state_dir):
                os.makedirs(state_dir)
            with open(self.state_file, "w", encoding="utf-8") as handle:
                yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=True)
        except Exception as exc:
            rospy.logerr("failed to write meter state file %s: %s", self.state_file, exc)


if __name__ == "__main__":
    try:
        MeterStateStoreNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
