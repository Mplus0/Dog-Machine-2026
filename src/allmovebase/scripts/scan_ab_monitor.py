#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Print compact raw/marking/clearing LaserScan and attitude statistics."""

import math
import statistics
import threading

import rospy
from sensor_msgs.msg import Imu, LaserScan


class ScanABMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._raw = None
        self._filtered = None
        self._clearing = None
        self._imu = None
        raw_topic = rospy.get_param("~raw_scan_topic", "/scan")
        filtered_topic = rospy.get_param(
            "~filtered_scan_topic", "/scan_ground_filtered"
        )
        clearing_topic = rospy.get_param(
            "~clearing_scan_topic", "/scan_ground_clearing"
        )
        imu_topic = rospy.get_param("~imu_topic", "/imu/data_throttled")
        self._print_period = float(rospy.get_param("~print_period", 1.0))
        rospy.Subscriber(raw_topic, LaserScan, self._raw_callback, queue_size=1)
        rospy.Subscriber(filtered_topic, LaserScan, self._filtered_callback, queue_size=1)
        rospy.Subscriber(clearing_topic, LaserScan, self._clearing_callback, queue_size=1)
        rospy.Subscriber(imu_topic, Imu, self._imu_callback, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(self._print_period), self._report)
        rospy.loginfo(
            "scan monitor: raw=%s filtered=%s clearing=%s imu=%s",
            raw_topic,
            filtered_topic,
            clearing_topic,
            imu_topic,
        )

    def _raw_callback(self, message):
        with self._lock:
            self._raw = message

    def _filtered_callback(self, message):
        with self._lock:
            self._filtered = message

    def _clearing_callback(self, message):
        with self._lock:
            self._clearing = message

    def _imu_callback(self, message):
        with self._lock:
            self._imu = message

    @staticmethod
    def _attitude_degrees(message):
        if message is None:
            return float("nan"), float("nan")
        q = message.orientation
        sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
        cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        return math.degrees(roll), math.degrees(pitch)

    @staticmethod
    def _scan_summary(message):
        if message is None:
            return "waiting"
        values = [
            value
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        if not values:
            return "valid=0/{:d} min=nan median=nan".format(len(message.ranges))
        return "valid={:d}/{:d} min={:.2f} median={:.2f}".format(
            len(values), len(message.ranges), min(values), statistics.median(values)
        )

    def _report(self, _event):
        with self._lock:
            raw = self._raw
            filtered = self._filtered
            clearing = self._clearing
            imu = self._imu
        roll, pitch = self._attitude_degrees(imu)
        rospy.loginfo(
            "roll=%6.2f pitch=%6.2f | raw: %s | filtered: %s | clearing: %s",
            roll,
            pitch,
            self._scan_summary(raw),
            self._scan_summary(filtered),
            self._scan_summary(clearing),
        )


def main():
    rospy.init_node("scan_ab_monitor", anonymous=False)
    ScanABMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
