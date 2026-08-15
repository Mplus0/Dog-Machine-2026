#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import uuid

import rospy
from std_msgs.msg import Bool, String


class DogArmTaskClient(object):
    def __init__(
        self,
        task_cmd_topic="/dog_arm/task_cmd",
        task_result_topic="/dog_arm/task_result",
        base_adjust_event_topic="/dog_arm/base_adjust_event",
        transport_connected_topic="/dog_arm/transport_connected",
        task_id_prefix="dog",
        wait_for_connection_timeout=5.0,
        require_transport_connected=True,
        base_adjust_event_timeout=5.0,
        base_adjust_settle_sec=1.0,
        max_pick_adjust_retries=1,
        report=None,
    ):
        self.task_cmd_topic = task_cmd_topic
        self.task_result_topic = task_result_topic
        self.base_adjust_event_topic = base_adjust_event_topic
        self.transport_connected_topic = transport_connected_topic
        self.task_id_prefix = task_id_prefix
        self.wait_for_connection_timeout = float(wait_for_connection_timeout)
        self.require_transport_connected = bool(require_transport_connected)
        self.base_adjust_event_timeout = float(base_adjust_event_timeout)
        self.base_adjust_settle_sec = float(base_adjust_settle_sec)
        self.max_pick_adjust_retries = int(max_pick_adjust_retries)
        self.report = report or rospy.loginfo

        self._seq = 1
        self._session_token = uuid.uuid4().hex[:8]
        self._lock = threading.Condition()
        self._results = {}
        self._base_adjust_events = {}
        self._transport_connected = not self.require_transport_connected

        self.task_cmd_pub = rospy.Publisher(self.task_cmd_topic, String, queue_size=10)
        self.result_sub = rospy.Subscriber(self.task_result_topic, String, self._on_result, queue_size=10)
        self.base_adjust_sub = rospy.Subscriber(
            self.base_adjust_event_topic,
            String,
            self._on_base_adjust_event,
            queue_size=10,
        )
        self.transport_sub = rospy.Subscriber(
            self.transport_connected_topic,
            Bool,
            self._on_transport_connected,
            queue_size=1,
        )

    def _next_task_id(self, label):
        safe = "".join(ch if ch.isalnum() else "_" for ch in str(label))[:24].strip("_")
        task_id = "%s_%s_%s_%04d" % (
            self.task_id_prefix,
            self._session_token,
            safe or "task",
            self._seq,
        )
        self._seq += 1
        return task_id

    def _decode_json(self, text, label):
        try:
            data = json.loads(text)
        except ValueError as exc:
            rospy.logwarn("invalid %s JSON: %s; raw=%s", label, exc, text)
            return None
        if not isinstance(data, dict):
            rospy.logwarn("invalid %s JSON object: %s", label, text)
            return None
        return data

    def _on_result(self, msg):
        data = self._decode_json(msg.data, "dog arm result")
        if data is None:
            return
        task_id = str(data.get("task_id", ""))
        if not task_id:
            rospy.logwarn("dog arm result missing task_id: %s", msg.data)
            return
        with self._lock:
            self._results[task_id] = data
            self._lock.notify_all()

    def _on_base_adjust_event(self, msg):
        data = self._decode_json(msg.data, "dog arm base adjust event")
        if data is None:
            return
        task_id = str(data.get("task_id", ""))
        if not task_id:
            return
        with self._lock:
            self._base_adjust_events[task_id] = data
            self._lock.notify_all()

    def _on_transport_connected(self, msg):
        with self._lock:
            self._transport_connected = bool(msg.data)
            self._lock.notify_all()

    def wait_for_server(self, timeout=None):
        timeout = self.wait_for_connection_timeout if timeout is None else float(timeout)
        if timeout <= 0.0:
            return False
        start = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            has_subscriber = self.task_cmd_pub.get_num_connections() > 0
            with self._lock:
                transport_ready = self._transport_connected
            if has_subscriber and transport_ready:
                return True
            if time.time() - start > timeout:
                return False
            rate.sleep()
        return False

    def _publish_task(self, task_id, cmd):
        cmd = self._normalize_cmd(cmd)
        payload = json.dumps(
            {
                "task_id": task_id,
                "cmd": cmd,
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        self.report("dog arm send: %s" % payload)
        self.task_cmd_pub.publish(String(data=payload))

    def _normalize_cmd(self, cmd):
        text = str(cmd).strip()
        for separator in (":", ","):
            text = text.replace(separator, " ")
        parts = [part for part in text.split() if part]
        if not parts:
            return ""
        aliases = {
            "place": "place_to_zone",
            "place_zone": "place_to_zone",
        }
        return aliases.get(parts[0], parts[0])

    def _wait_for_result(self, task_id, timeout):
        deadline = time.time() + max(0.0, float(timeout))
        with self._lock:
            while not rospy.is_shutdown():
                result = self._results.pop(task_id, None)
                if result is not None:
                    return result
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None
                self._lock.wait(min(0.2, remaining))
        return None

    def _wait_for_base_adjust_event(self, task_id, timeout):
        deadline = time.time() + max(0.0, float(timeout))
        with self._lock:
            while not rospy.is_shutdown():
                event = self._base_adjust_events.pop(task_id, None)
                if event is not None:
                    return event
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    return None
                self._lock.wait(min(0.2, remaining))
        return None

    def run_task(self, cmd, label, timeout):
        cmd = self._normalize_cmd(cmd)
        task_id = self._next_task_id(label)
        self._publish_task(task_id, cmd)
        result = self._wait_for_result(task_id, timeout)
        if result is None:
            self.report("dog arm timeout: %s task_id=%s timeout=%.1fs" % (label, task_id, timeout))
            return False, {"task_id": task_id, "result": "%s_timeout" % cmd}
        self.report("dog arm result: %s" % json.dumps(result, ensure_ascii=False, sort_keys=True))
        ok_result = "pick_success" if cmd == "pick" else "place_success"
        return result.get("result") == ok_result, result

    def pick_with_adjust_retry(self, label, timeout):
        attempts = max(0, self.max_pick_adjust_retries) + 1
        for attempt in range(1, attempts + 1):
            task_id = self._next_task_id("%s_pick" % label)
            self._publish_task(task_id, "pick")
            result = self._wait_for_result(task_id, timeout)
            if result is None:
                self.report("dog arm pick timeout: %s task_id=%s timeout=%.1fs" % (label, task_id, timeout))
                return False

            self.report("dog arm pick result: %s" % json.dumps(result, ensure_ascii=False, sort_keys=True))
            if result.get("result") == "pick_success":
                return True

            if result.get("error") != "need_base_adjust" or attempt >= attempts:
                return False

            event = self._wait_for_base_adjust_event(task_id, self.base_adjust_event_timeout)
            if event is None:
                self.report("dog arm base adjust completion timeout: task_id=%s" % task_id)
                return False
            self.report("dog arm base adjust event: %s" % json.dumps(event, ensure_ascii=False, sort_keys=True))
            if not bool(event.get("executed", False)):
                self.report("dog arm pick retry refused: base adjust was not executed")
                return False
            self.report("dog arm pick retry after base adjust, attempt %d/%d" % (attempt + 1, attempts))
            if self.base_adjust_settle_sec > 0.0:
                rospy.sleep(self.base_adjust_settle_sec)
        return False

    def place_to_zone(self, label, timeout):
        ok, _ = self.run_task("place_to_zone", "%s_place" % label, timeout)
        return ok
