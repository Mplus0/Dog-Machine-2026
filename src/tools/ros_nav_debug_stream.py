#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


class SharedJpegFrame:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg = None
        self._updated_at = 0.0

    def update_image(self, image, jpeg_quality):
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
        )
        if ok:
            with self._lock:
                self._jpg = encoded.tobytes()
                self._updated_at = time.time()

    def snapshot(self):
        with self._lock:
            return self._jpg, self._updated_at


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavDebugRenderer:
    def __init__(self, args):
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import Image, LaserScan

        self.rospy = rospy
        self.bridge = CvBridge()
        self.args = args
        self.depth_frame = SharedJpegFrame()
        self.map_frame = SharedJpegFrame()
        self.status_lock = threading.Lock()
        self.status = {
            "depth": "no_frame",
            "map": "no_map",
            "amcl": "no_pose",
            "scan": "no_scan",
            "move_base": "no_status",
        }

        self.map_msg = None
        self.map_image = None
        self.pose_msg = None
        self.particles_msg = None
        self.scan_msg = None
        self.global_plan_msg = None
        self.local_plan_msg = None
        self.move_base_status = None

        rospy.Subscriber(args.depth_topic, Image, self._on_depth, queue_size=1)
        rospy.Subscriber(args.map_topic, OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber(args.amcl_pose_topic, PoseWithCovarianceStamped, self._on_amcl_pose, queue_size=1)
        rospy.Subscriber(args.particle_topic, PoseArray, self._on_particles, queue_size=1)
        rospy.Subscriber(args.scan_topic, LaserScan, self._on_scan, queue_size=1)
        rospy.Subscriber(args.global_plan_topic, Path, self._on_global_plan, queue_size=1)
        rospy.Subscriber(args.local_plan_topic, Path, self._on_local_plan, queue_size=1)
        rospy.Subscriber(args.move_base_status_topic, GoalStatusArray, self._on_move_base_status, queue_size=1)

    def _set_status(self, key, value):
        with self.status_lock:
            self.status[key] = value

    def status_text(self):
        with self.status_lock:
            return "\n".join("%s=%s" % (key, self.status[key]) for key in sorted(self.status))

    def _on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            image = self._render_depth(depth)
            self.depth_frame.update_image(image, self.args.jpeg_quality)
            self._set_status("depth", "ok %.3f" % time.time())
        except Exception as exc:
            self._set_status("depth", "error %s" % exc)

    def _render_depth(self, depth):
        depth_m = depth.astype(np.float32)
        if str(depth.dtype).startswith("uint"):
            depth_m *= self.args.depth_scale

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        valid &= depth_m >= self.args.depth_min
        valid &= depth_m <= self.args.depth_max
        clipped = np.zeros_like(depth_m, dtype=np.float32)
        clipped[valid] = depth_m[valid]

        normalized = np.zeros_like(depth_m, dtype=np.uint8)
        if np.any(valid):
            normalized[valid] = np.clip(
                255.0 * (1.0 - (clipped[valid] - self.args.depth_min) / max(0.001, self.args.depth_max - self.args.depth_min)),
                0,
                255,
            ).astype(np.uint8)
        color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        color[~valid] = (20, 20, 20)
        text = "depth %s, range %.2f-%.2fm" % (str(depth.shape), self.args.depth_min, self.args.depth_max)
        cv2.putText(color, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return color

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_image = self._occupancy_to_image(msg)
        self._set_status("map", "ok %dx%d res=%.3f" % (msg.info.width, msg.info.height, msg.info.resolution))
        self._update_map_frame()

    def _occupancy_to_image(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[data < 0] = (170, 170, 170)
        image[data == 0] = (245, 245, 245)
        occupied = data > 50
        image[occupied] = (35, 35, 35)
        mid = (data > 0) & (data <= 50)
        image[mid] = (120, 120, 120)
        return cv2.flip(image, 0)

    def _on_amcl_pose(self, msg):
        self.pose_msg = msg
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._set_status("amcl", "x=%.3f y=%.3f yaw=%.1fdeg" % (p.x, p.y, math.degrees(yaw)))
        self._update_map_frame()

    def _on_particles(self, msg):
        self.particles_msg = msg
        self._update_map_frame()

    def _on_scan(self, msg):
        self.scan_msg = msg
        finite = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        if finite:
            self._set_status("scan", "ok min=%.3f count=%d" % (min(finite), len(finite)))
        else:
            self._set_status("scan", "no_valid_ranges")

    def _on_global_plan(self, msg):
        self.global_plan_msg = msg
        self._update_map_frame()

    def _on_local_plan(self, msg):
        self.local_plan_msg = msg
        self._update_map_frame()

    def _on_move_base_status(self, msg):
        if msg.status_list:
            item = msg.status_list[-1]
            self._set_status("move_base", "status=%d text=%s" % (item.status, item.text))
        else:
            self._set_status("move_base", "empty")

    def world_to_pixel(self, x, y):
        if self.map_msg is None:
            return None
        info = self.map_msg.info
        px = int(round((x - info.origin.position.x) / info.resolution))
        py = int(round((y - info.origin.position.y) / info.resolution))
        if px < 0 or py < 0 or px >= info.width or py >= info.height:
            return None
        return px, info.height - 1 - py

    def _draw_path(self, image, path_msg, color, max_points=800):
        if path_msg is None or not path_msg.poses:
            return
        points = []
        stride = max(1, len(path_msg.poses) // max_points)
        for pose_stamped in path_msg.poses[::stride]:
            p = pose_stamped.pose.position
            pixel = self.world_to_pixel(p.x, p.y)
            if pixel is not None:
                points.append(pixel)
        if len(points) >= 2:
            cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, 2, cv2.LINE_AA)

    def _draw_particles(self, image):
        if self.particles_msg is None:
            return
        poses = self.particles_msg.poses
        stride = max(1, len(poses) // self.args.max_particles_draw)
        for pose in poses[::stride]:
            p = pose.position
            pixel = self.world_to_pixel(p.x, p.y)
            if pixel is not None:
                cv2.circle(image, pixel, 1, (0, 80, 255), -1)

    def _draw_amcl_pose(self, image):
        if self.pose_msg is None:
            return
        pose = self.pose_msg.pose.pose
        p = pose.position
        pixel = self.world_to_pixel(p.x, p.y)
        if pixel is None:
            return
        yaw = yaw_from_quaternion(pose.orientation)
        length = max(12, int(0.35 / self.map_msg.info.resolution))
        end = (
            int(round(pixel[0] + math.cos(yaw) * length)),
            int(round(pixel[1] - math.sin(yaw) * length)),
        )
        cv2.circle(image, pixel, 5, (0, 220, 255), -1)
        cv2.arrowedLine(image, pixel, end, (0, 220, 255), 3, cv2.LINE_AA, tipLength=0.35)

    def _update_map_frame(self):
        if self.map_image is None:
            return
        image = self.map_image.copy()
        self._draw_path(image, self.global_plan_msg, (0, 180, 0))
        self._draw_path(image, self.local_plan_msg, (0, 120, 255))
        self._draw_particles(image)
        self._draw_amcl_pose(image)
        label = "map=%s pose=%s" % (self.args.map_topic, self.args.amcl_pose_topic)
        cv2.putText(image, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (30, 30, 30), 2, cv2.LINE_AA)
        if self.args.map_scale != 1.0:
            image = cv2.resize(image, None, fx=self.args.map_scale, fy=self.args.map_scale, interpolation=cv2.INTER_NEAREST)
        self.map_frame.update_image(image, self.args.jpeg_quality)


def build_handler(renderer, stream_fps):
    frame_interval = 1.0 / max(stream_fps, 1)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send_index()
            elif self.path == "/depth.mjpg":
                self._send_mjpeg(renderer.depth_frame)
            elif self.path == "/map.mjpg":
                self._send_mjpeg(renderer.map_frame)
            elif self.path == "/health":
                self._send_health()
            else:
                self.send_error(404)

        def _send_index(self):
            html = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROS Nav Debug</title>
  <style>
    body { margin: 0; background: #151515; color: #eee; font-family: Arial, sans-serif; }
    header { padding: 12px 16px; border-bottom: 1px solid #333; }
    main { display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); gap: 12px; padding: 12px; }
    section { background: #202020; border: 1px solid #333; padding: 10px; }
    h2 { margin: 0 0 8px; font-size: 16px; font-weight: 600; }
    img { width: 100%; height: auto; background: #000; display: block; }
    a { color: #8fd3ff; }
  </style>
</head>
<body>
  <header>
    <strong>ROS Nav Debug</strong>
    <span> | </span>
    <a href="/health">health</a>
  </header>
  <main>
    <section>
      <h2>D435i Depth</h2>
      <img src="/depth.mjpg" alt="depth stream">
    </section>
    <section>
      <h2>Map + AMCL</h2>
      <img src="/map.mjpg" alt="map stream">
    </section>
  </main>
</body>
</html>"""
            body = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_health(self):
            body = (renderer.status_text() + "\n").encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_mjpeg(self, shared_frame):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                jpg, _ = shared_frame.snapshot()
                if jpg is None:
                    time.sleep(frame_interval)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(frame_interval)

    return Handler


def parse_args():
    parser = argparse.ArgumentParser(description="Lightweight browser dashboard for ROS navigation debugging.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--stream-fps", type=int, default=5)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--depth-min", type=float, default=0.2)
    parser.add_argument("--depth-max", type=float, default=3.0)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--amcl-pose-topic", default="/amcl_pose")
    parser.add_argument("--particle-topic", default="/particlecloud")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--global-plan-topic", default="/move_base/NavfnROS/plan")
    parser.add_argument("--local-plan-topic", default="/move_base/TebLocalPlannerROS/local_plan")
    parser.add_argument("--move-base-status-topic", default="/move_base/status")
    parser.add_argument("--map-scale", type=float, default=2.0)
    parser.add_argument("--max-particles-draw", type=int, default=600)
    return parser.parse_args()


def main():
    import rospy

    args = parse_args()
    rospy.init_node("ros_nav_debug_stream", anonymous=False)

    renderer = NavDebugRenderer(args)
    server = ThreadingHTTPServer((args.host, args.port), build_handler(renderer, args.stream_fps))

    stop_event = threading.Event()

    def stop(_signum=None, _frame=None):
        stop_event.set()
        server.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print("ROS nav debug stream: http://%s:%d" % (args.host, args.port), flush=True)
    print("open http://<Jetson-IP>:%d from your PC browser" % args.port, flush=True)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown() and not stop_event.is_set():
        rate.sleep()

    server.server_close()


if __name__ == "__main__":
    main()
