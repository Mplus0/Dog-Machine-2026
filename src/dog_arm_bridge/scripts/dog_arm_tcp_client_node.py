#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Full-duplex TCP transport between the ROS1 dog and the ROS2 arm.

The wire format is newline-delimited JSON.  Application payloads remain the
existing std_msgs/String JSON objects; the transport only adds an envelope,
acknowledgement, heartbeat and duplicate suppression.
"""

from collections import OrderedDict
import hashlib
import hmac
import json
import os
import socket
import threading
import time
import uuid

import rospy
from std_msgs.msg import Bool, String


PROTOCOL_VERSION = 1


class DogArmTcpClientNode(object):
    def __init__(self):
        rospy.init_node("dog_arm_tcp_client", anonymous=False)

        self.server_host = str(rospy.get_param("~server_host", "192.168.31.56")).strip()
        self.server_port = int(rospy.get_param("~server_port", 47001))
        self.connect_timeout = max(0.2, float(rospy.get_param("~connect_timeout", 3.0)))
        self.reconnect_initial = max(0.1, float(rospy.get_param("~reconnect_initial", 0.5)))
        self.reconnect_max = max(self.reconnect_initial, float(rospy.get_param("~reconnect_max", 5.0)))
        self.heartbeat_interval = max(0.2, float(rospy.get_param("~heartbeat_interval", 1.0)))
        self.heartbeat_timeout = max(
            self.heartbeat_interval * 2.0,
            float(rospy.get_param("~heartbeat_timeout", 5.0)),
        )
        self.handshake_timeout = max(0.5, float(rospy.get_param("~handshake_timeout", 3.0)))
        self.resend_interval = max(0.1, float(rospy.get_param("~resend_interval", 0.8)))
        self.outbound_ttl = max(1.0, float(rospy.get_param("~outbound_ttl", 15.0)))
        self.max_frame_bytes = max(1024, int(rospy.get_param("~max_frame_bytes", 65536)))
        self.dedupe_cache_size = max(16, int(rospy.get_param("~dedupe_cache_size", 512)))
        self.shared_secret_file = os.path.expanduser(
            str(rospy.get_param("~shared_secret_file", "~/.ros/dog_arm_shared_secret"))
        )
        self._shared_secret = self._load_shared_secret(self.shared_secret_file)

        self.task_cmd_topic = rospy.get_param("~task_cmd_topic", "/dog_arm/task_cmd")
        self.task_result_topic = rospy.get_param("~task_result_topic", "/dog_arm/task_result")
        self.base_adjust_req_topic = rospy.get_param("~base_adjust_req_topic", "/dog_arm/base_adjust_req")
        self.connected_topic = rospy.get_param("~connected_topic", "/dog_arm/transport_connected")
        self.status_topic = rospy.get_param("~status_topic", "/dog_arm/transport_status")

        self.session_id = "dog-%s" % uuid.uuid4().hex[:12]
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._pending = OrderedDict()
        self._seen_inbound = OrderedDict()
        self._connected = False
        self._socket = None

        self.result_pub = rospy.Publisher(self.task_result_topic, String, queue_size=10)
        self.base_adjust_pub = rospy.Publisher(self.base_adjust_req_topic, String, queue_size=10)
        self.connected_pub = rospy.Publisher(self.connected_topic, Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.task_sub = rospy.Subscriber(self.task_cmd_topic, String, self._on_task_cmd, queue_size=10)

        self.connected_pub.publish(Bool(data=False))
        self._publish_status("disconnected", "starting")
        rospy.on_shutdown(self.shutdown)
        self._worker = threading.Thread(target=self._run, name="dog_arm_tcp_client")
        self._worker.daemon = True
        self._worker.start()

        rospy.loginfo(
            "dog-arm TCP client started: server=%s:%d task=%s result=%s adjust=%s",
            self.server_host,
            self.server_port,
            self.task_cmd_topic,
            self.task_result_topic,
            self.base_adjust_req_topic,
        )

    def _encode(self, value):
        return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")

    def _decode_payload(self, text, label):
        try:
            value = json.loads(text)
        except ValueError as exc:
            rospy.logwarn("invalid %s JSON: %s; raw=%s", label, exc, text)
            return None
        if not isinstance(value, dict):
            rospy.logwarn("invalid %s: expected JSON object; raw=%s", label, text)
            return None
        return value

    def _load_shared_secret(self, path):
        try:
            with open(path, "rb") as handle:
                secret = handle.read().strip()
        except OSError as exc:
            raise rospy.ROSInitException("cannot read dog-arm shared secret %s: %s" % (path, exc))
        if len(secret) < 16:
            raise rospy.ROSInitException("dog-arm shared secret must contain at least 16 bytes: %s" % path)
        return secret

    def _proof(self, role, client_nonce, server_nonce):
        text = "%s|%s|%s|%d" % (role, client_nonce, server_nonce, PROTOCOL_VERSION)
        return hmac.new(self._shared_secret, text.encode("utf-8"), hashlib.sha256).hexdigest()

    def _sign_envelope(self, envelope):
        signed = dict(envelope)
        signed.pop("signature", None)
        canonical = json.dumps(
            signed,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        signed["signature"] = hmac.new(self._shared_secret, canonical, hashlib.sha256).hexdigest()
        return signed

    def _verify_envelope(self, envelope):
        signature = str(envelope.get("signature", ""))
        if not signature:
            return False
        unsigned = dict(envelope)
        unsigned.pop("signature", None)
        canonical = json.dumps(
            unsigned,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        expected = hmac.new(self._shared_secret, canonical, hashlib.sha256).hexdigest()
        return hmac.compare_digest(signature, expected)

    def _on_task_cmd(self, msg):
        payload = self._decode_payload(msg.data, "task command")
        if payload is None:
            return
        task_id = str(payload.get("task_id", "")).strip()
        cmd = str(payload.get("cmd", "")).strip()
        if not task_id or not cmd:
            rospy.logwarn("task command missing task_id/cmd: %s", msg.data)
            return
        message_id = "task_cmd:%s" % task_id
        envelope = self._envelope("task_cmd", payload, message_id)
        now = time.monotonic()
        with self._lock:
            if message_id in self._pending:
                rospy.logwarn("duplicate pending task ignored: %s", task_id)
                return
            self._pending[message_id] = {
                "envelope": envelope,
                "created": now,
                "last_send": 0.0,
            }
        rospy.loginfo("queued dog -> arm task: task_id=%s cmd=%s", task_id, cmd)

    def _envelope(self, message_type, payload=None, message_id=None):
        value = {
            "version": PROTOCOL_VERSION,
            "type": message_type,
            "session_id": self.session_id,
            "timestamp": time.time(),
        }
        if message_id:
            value["message_id"] = message_id
        if payload is not None:
            value["payload"] = payload
        return value

    def _publish_status(self, state, detail=""):
        payload = {
            "state": state,
            "detail": str(detail),
            "peer": "%s:%d" % (self.server_host, self.server_port),
            "session_id": self.session_id,
            "timestamp": time.time(),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _set_connected(self, connected, detail=""):
        changed = False
        with self._lock:
            connected = bool(connected)
            if self._connected != connected:
                self._connected = connected
                changed = True
        if changed:
            self.connected_pub.publish(Bool(data=connected))
            self._publish_status("connected" if connected else "disconnected", detail)
            log = rospy.loginfo if connected else rospy.logwarn
            log("dog-arm TCP transport %s: %s", "connected" if connected else "disconnected", detail)

    def _remember_inbound(self, message_id):
        with self._lock:
            duplicate = message_id in self._seen_inbound
            self._seen_inbound[message_id] = time.monotonic()
            self._seen_inbound.move_to_end(message_id)
            while len(self._seen_inbound) > self.dedupe_cache_size:
                self._seen_inbound.popitem(last=False)
        return duplicate

    def _ack(self, sock, message_id):
        self._send(sock, self._envelope("ack", {"ack_id": message_id}))

    def _handle_inbound(self, sock, envelope, handshake):
        if not self._verify_envelope(envelope):
            raise RuntimeError("arm TCP frame signature verification failed")
        if int(envelope.get("version", -1)) != PROTOCOL_VERSION:
            raise RuntimeError("unsupported protocol version: %s" % envelope.get("version"))
        message_type = str(envelope.get("type", ""))

        if message_type == "challenge":
            client_nonce = str(envelope.get("client_nonce", ""))
            server_nonce = str(envelope.get("server_nonce", ""))
            if client_nonce != handshake["client_nonce"] or not server_nonce:
                raise RuntimeError("invalid arm authentication challenge")
            handshake["server_nonce"] = server_nonce
            auth = self._envelope("auth")
            auth["client_nonce"] = client_nonce
            auth["server_nonce"] = server_nonce
            auth["proof"] = self._proof("dog", client_nonce, server_nonce)
            self._send(sock, auth)
            return
        if message_type == "hello_ack":
            if str(envelope.get("role", "")) != "arm":
                raise RuntimeError("unexpected TCP peer role")
            expected = self._proof("arm", handshake["client_nonce"], handshake["server_nonce"])
            if not hmac.compare_digest(str(envelope.get("proof", "")), expected):
                raise RuntimeError("arm authentication proof mismatch")
            handshake["complete"] = True
            return
        if not handshake["complete"]:
            raise RuntimeError("message received before arm authentication")
        if message_type == "heartbeat":
            self._send(sock, self._envelope("heartbeat_ack"))
            return
        if message_type == "heartbeat_ack":
            return
        if message_type == "ack":
            payload = envelope.get("payload", {})
            ack_id = str(payload.get("ack_id", "")) if isinstance(payload, dict) else ""
            if ack_id:
                with self._lock:
                    self._pending.pop(ack_id, None)
            return
        if message_type not in ("task_result", "base_adjust_req"):
            rospy.logwarn("ignored unknown arm TCP message type: %s", message_type)
            return

        message_id = str(envelope.get("message_id", "")).strip()
        payload = envelope.get("payload")
        if not message_id or not isinstance(payload, dict):
            rospy.logwarn("ignored invalid %s envelope", message_type)
            return
        self._ack(sock, message_id)
        if self._remember_inbound(message_id):
            rospy.loginfo("duplicate arm message acknowledged without republish: %s", message_id)
            return

        text = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if message_type == "task_result":
            self.result_pub.publish(String(data=text))
        else:
            self.base_adjust_pub.publish(String(data=text))
        rospy.loginfo("arm -> dog %s: %s", message_type, text)

    def _send(self, sock, envelope):
        sock.sendall(self._encode(self._sign_envelope(envelope)))

    def _send_pending(self, sock, now):
        expired = []
        due = []
        with self._lock:
            for message_id, item in list(self._pending.items()):
                age = now - item["created"]
                if age > self.outbound_ttl:
                    expired.append((message_id, item["envelope"]))
                elif now - item["last_send"] >= self.resend_interval:
                    due.append((message_id, item["envelope"]))
            for message_id, _ in expired:
                self._pending.pop(message_id, None)
        for message_id, envelope in expired:
            rospy.logerr("dropping unacknowledged stale arm task: %s", message_id)
            payload = envelope.get("payload", {})
            cmd = str(payload.get("cmd", "")) if isinstance(payload, dict) else ""
            task_id = str(payload.get("task_id", "")) if isinstance(payload, dict) else ""
            if task_id:
                failed = "pick_failed" if cmd == "pick" else "place_failed"
                result = {
                    "task_id": task_id,
                    "result": failed,
                    "error": "transport_delivery_timeout",
                }
                self.result_pub.publish(
                    String(data=json.dumps(result, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
                )
        for message_id, envelope in due:
            self._send(sock, envelope)
            with self._lock:
                if message_id in self._pending:
                    self._pending[message_id]["last_send"] = now

    def _connection_loop(self, sock):
        sock.settimeout(0.15)
        buffer = b""
        handshake = {
            "complete": False,
            "client_nonce": uuid.uuid4().hex,
            "server_nonce": "",
        }
        connected_at = time.monotonic()
        last_rx = connected_at
        last_heartbeat = 0.0
        hello = self._envelope("hello")
        hello["role"] = "dog"
        hello["client_nonce"] = handshake["client_nonce"]
        self._send(sock, hello)

        while not self._stop.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("peer closed connection")
                buffer += chunk
                last_rx = now
                if len(buffer) > self.max_frame_bytes and b"\n" not in buffer:
                    raise RuntimeError("incoming frame exceeds max_frame_bytes")
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    if not raw:
                        continue
                    if len(raw) > self.max_frame_bytes:
                        raise RuntimeError("incoming frame exceeds max_frame_bytes")
                    envelope = json.loads(raw.decode("utf-8"))
                    if not isinstance(envelope, dict):
                        raise RuntimeError("incoming frame is not a JSON object")
                    self._handle_inbound(sock, envelope, handshake)
            except socket.timeout:
                pass

            now = time.monotonic()
            if not handshake["complete"]:
                if now - connected_at > self.handshake_timeout:
                    raise RuntimeError("arm handshake timeout")
                continue
            self._set_connected(True, "handshake complete")
            self._send_pending(sock, now)
            if now - last_heartbeat >= self.heartbeat_interval:
                self._send(sock, self._envelope("heartbeat"))
                last_heartbeat = now
            if now - last_rx > self.heartbeat_timeout:
                raise RuntimeError("arm heartbeat timeout")

    def _run(self):
        delay = self.reconnect_initial
        while not self._stop.is_set() and not rospy.is_shutdown():
            sock = None
            try:
                self._publish_status("connecting", "%s:%d" % (self.server_host, self.server_port))
                sock = socket.create_connection(
                    (self.server_host, self.server_port),
                    timeout=self.connect_timeout,
                )
                with self._lock:
                    self._socket = sock
                delay = self.reconnect_initial
                self._connection_loop(sock)
            except Exception as exc:
                self._set_connected(False, str(exc))
                if not self._stop.is_set() and not rospy.is_shutdown():
                    rospy.logwarn_throttle(5.0, "dog-arm TCP connect/IO error: %s", exc)
            finally:
                with self._lock:
                    self._socket = None
                if sock is not None:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    try:
                        sock.close()
                    except Exception:
                        pass
                self._set_connected(False, "connection closed")
            if self._stop.wait(delay):
                break
            delay = min(self.reconnect_max, delay * 2.0)

    def shutdown(self):
        self._stop.set()
        with self._lock:
            sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        self._set_connected(False, "shutdown")

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        DogArmTcpClientNode().spin()
    except rospy.ROSInterruptException:
        pass
