#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rospy


class TaskBudget(object):
    """Lightweight wall-clock budget helper for task-level decisions."""

    def __init__(self, enabled=False, total_sec=300.0, start_now=True, warn_interval=5.0):
        self.enabled = bool(enabled)
        self.total_sec = float(total_sec)
        self.warn_interval = float(warn_interval)
        self._start_time = time.time() if start_now else None
        self._last_warn = {}

    @classmethod
    def from_params(cls, prefix="~task_budget", default_enabled=False,
                    default_total_sec=300.0, start_now=True):
        enabled = cls._param_bool(prefix + "_enable", default_enabled)
        total_sec = float(rospy.get_param(prefix + "_total", default_total_sec))
        warn_interval = float(rospy.get_param(prefix + "_warn_interval", 5.0))
        return cls(enabled, total_sec, start_now=start_now, warn_interval=warn_interval)

    @staticmethod
    def _param_bool(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def start(self):
        if self.enabled and self._start_time is None:
            self._start_time = time.time()

    def is_started(self):
        return self._start_time is not None

    def elapsed(self):
        if not self.enabled or self._start_time is None:
            return 0.0
        return max(0.0, time.time() - self._start_time)

    def remaining(self):
        if not self.enabled or self._start_time is None:
            return float("inf")
        return max(0.0, self.total_sec - self.elapsed())

    def expired(self):
        return self.enabled and self._start_time is not None and self.remaining() <= 0.0

    def allow(self, min_remaining=0.0):
        if not self.enabled or self._start_time is None:
            return True
        return self.remaining() >= max(0.0, float(min_remaining))

    def check(self, label, min_remaining=0.0):
        if self.allow(min_remaining):
            return True
        self.warn(label, "budget low: remaining=%.1fs, required=%.1fs" %
                  (self.remaining(), max(0.0, float(min_remaining))))
        return False

    def cap_timeout(self, requested, reserve=0.0, minimum=0.0):
        requested = max(0.0, float(requested))
        if not self.enabled or self._start_time is None:
            return requested
        available = self.remaining() - max(0.0, float(reserve))
        if available <= 0.0:
            return 0.0
        if requested < minimum:
            return requested
        return max(0.0, min(requested, available))

    def sleep(self, duration, reserve=0.0):
        timeout = self.cap_timeout(duration, reserve=reserve)
        if timeout > 0.0:
            rospy.sleep(timeout)
        return timeout >= max(0.0, float(duration))

    def warn(self, label, message):
        now = time.time()
        key = str(label)
        last = self._last_warn.get(key, 0.0)
        if now - last >= self.warn_interval:
            rospy.logwarn("[task_budget] %s: %s", label, message)
            self._last_warn[key] = now

    def log_state(self, label):
        if self.enabled and self._start_time is not None:
            rospy.loginfo("[task_budget] %s: elapsed=%.1fs remaining=%.1fs total=%.1fs",
                          label, self.elapsed(), self.remaining(), self.total_sec)
