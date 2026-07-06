# -*- coding: utf-8 -*-

import argparse
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


DEFAULT_RTSP_URL = "rtsp://192.168.1.120:8554/test"


class SharedJpegFrame:
    def __init__(self):
        self._lock = threading.Lock()
        self._jpg = None
        self._updated_at = 0.0

    def update(self, jpg):
        with self._lock:
            self._jpg = jpg
            self._updated_at = time.time()

    def snapshot(self):
        with self._lock:
            return self._jpg, self._updated_at


class OpenCvFrameSource:
    def __init__(self, camera_source, width=None, height=None, fps=None):
        self.camera_source = camera_source
        self.cap = None
        self.width = width
        self.height = height
        self.fps = fps

    def open(self):
        describe_camera_source(self.camera_source)
        print(f"trying OpenCV camera: {self.camera_source}", flush=True)

        self.cap = cv2.VideoCapture(self.camera_source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if self.width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width))
        if self.height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
        if self.fps:
            self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))

        if not self.cap.isOpened():
            self.close()
            raise RuntimeError(
                f"Could not open camera: {self.camera_source}. "
                f"Try --backend gstreamer, --camera-index N, or --camera /dev/videoX."
            )

        frame = self.read()
        if frame is None:
            self.close()
            raise RuntimeError(f"Camera opened but could not read frame: {self.camera_source}.")

        print(f"OpenCV camera opened: {self.camera_source}", flush=True)
        print(f"first frame shape: {frame.shape}", flush=True)

    def read(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class GStreamerRtspFrameSource:
    def __init__(self, rtsp_url, width, height):
        self.rtsp_url = rtsp_url
        self.width = width
        self.height = height
        self.pipeline = None

    def open(self):
        import gi
        import numpy as np

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self._gst = Gst
        self._np = np

        self.pipeline_str = (
            f"rtspsrc location={self.rtsp_url} latency=100 ! "
            "rtph264depay ! h264parse config-interval=1 ! "
            "nvv4l2decoder ! video/x-raw(memory:NVMM) ! "
            "nvvidconv ! video/x-raw,format=BGRx ! "
            f"videoscale ! video/x-raw,width={self.width},height={self.height} ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "queue max-size-buffers=5 max-size-bytes=0 max-size-time=50000000 leaky=downstream ! "
            "appsink name=sink sync=false max-buffers=1 drop=true"
        )

        print(f"trying GStreamer RTSP: {self.rtsp_url}", flush=True)
        print(f"pipeline: {self.pipeline_str}", flush=True)

        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.set_property("emit-signals", True)
        self.pipeline.set_state(Gst.State.PLAYING)

        frame = self.read(timeout_sec=5.0)
        if frame is None:
            self.close()
            raise RuntimeError(f"Could not read first RTSP frame from {self.rtsp_url}.")

        print(f"GStreamer RTSP opened: {self.rtsp_url}", flush=True)
        print(f"first frame shape: {frame.shape}", flush=True)

    def read(self, timeout_sec=1.0):
        if self.pipeline is None:
            return None

        sample = self.sink.emit("try-pull-sample", int(timeout_sec * 1000000000))
        if sample is None:
            return None

        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")
        buffer = sample.get_buffer()
        ok, map_info = buffer.map(self._gst.MapFlags.READ)
        if not ok:
            return None
        try:
            arr = self._np.frombuffer(map_info.data, dtype=self._np.uint8)
            return arr.reshape((height, width, 3)).copy()
        finally:
            buffer.unmap(map_info)

    def close(self):
        if self.pipeline is not None:
            self.pipeline.set_state(self._gst.State.NULL)
            self.pipeline = None


def describe_camera_source(camera_source):
    if isinstance(camera_source, int):
        print(f"camera configured source: index {camera_source}", flush=True)
        print(f"camera resolved path hint: /dev/video{camera_source}", flush=True)
        return

    exists = os.path.exists(camera_source)
    is_link = os.path.islink(camera_source)
    link_target = os.readlink(camera_source) if is_link else ""
    real_path = os.path.realpath(camera_source) if exists or is_link else ""

    print(f"camera configured path: {camera_source}", flush=True)
    print(f"camera path exists: {exists}", flush=True)
    print(f"camera path is symlink: {is_link}", flush=True)

    if is_link:
        print(f"camera symlink target: {link_target}", flush=True)
    if real_path:
        print(f"camera resolved path: {real_path}", flush=True)


def build_stream_handler(shared_frame, stream_fps):
    frame_interval = 1.0 / max(stream_fps, 1)

    class StreamHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._send_index()
            elif self.path == "/stream.mjpg":
                self._send_mjpeg()
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
  <title>Wide Angle Camera Stream</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: Arial, sans-serif; }
    main { min-height: 100vh; display: grid; place-items: center; padding: 16px; box-sizing: border-box; }
    img { width: min(100%, 1280px); height: auto; background: #000; }
  </style>
</head>
<body>
  <main>
    <img src="/stream.mjpg" alt="wide angle camera stream">
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
            jpg, updated_at = shared_frame.snapshot()
            if jpg is None:
                body = b"status=no_frame\n"
            else:
                body = f"status=ok, updated_at={updated_at:.3f}\n".encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_mjpeg(self):
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

    return StreamHandler


def start_stream_server(shared_frame, host, port, stream_fps):
    handler = build_stream_handler(shared_frame, stream_fps)
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def parse_args():
    parser = argparse.ArgumentParser(
        description="Preview the Lite3 wide-angle camera through an MJPEG web server."
    )
    parser.add_argument(
        "camera_index_pos",
        nargs="?",
        type=int,
        help="Optional OpenCV camera index, for example: python3 wide_angle_stream_test.py 0",
    )
    parser.add_argument(
        "--backend",
        choices=("gstreamer", "opencv"),
        default="gstreamer",
        help="Frame source backend. gstreamer matches last year's wide-angle RTSP path.",
    )
    parser.add_argument("--camera-index", type=int, default=None, help="OpenCV camera index.")
    parser.add_argument("--camera", default="/dev/video0", help="OpenCV camera device path.")
    parser.add_argument("--rtsp-url", default=DEFAULT_RTSP_URL, help="Wide-angle RTSP URL.")
    parser.add_argument("--width", type=int, default=1280, help="Requested frame width.")
    parser.add_argument("--height", type=int, default=720, help="Requested frame height.")
    parser.add_argument("--capture-fps", type=int, default=None, help="Requested OpenCV capture FPS.")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address.")
    parser.add_argument("--port", type=int, default=8081, help="HTTP stream port.")
    parser.add_argument("--stream-fps", type=int, default=10, help="MJPEG stream FPS.")
    parser.add_argument("--jpeg-quality", type=int, default=70, help="JPEG quality, 1-100.")
    return parser.parse_args()


def resolve_opencv_camera_source(args):
    if args.camera_index is not None:
        return args.camera_index
    if args.camera_index_pos is not None:
        return args.camera_index_pos
    return args.camera


def make_frame_source(args):
    if args.backend == "gstreamer":
        return GStreamerRtspFrameSource(args.rtsp_url, args.width, args.height)
    return OpenCvFrameSource(
        resolve_opencv_camera_source(args),
        width=args.width,
        height=args.height,
        fps=args.capture_fps,
    )


def main():
    args = parse_args()
    jpeg_quality = max(1, min(args.jpeg_quality, 100))

    stop_event = threading.Event()
    shared_frame = SharedJpegFrame()

    def stop(_signum=None, _frame=None):
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    server = start_stream_server(
        shared_frame=shared_frame,
        host=args.host,
        port=args.port,
        stream_fps=args.stream_fps,
    )

    print(f"stream server started: http://{args.host}:{args.port}", flush=True)
    print(f"open http://<device-ip>:{args.port} from your PC browser", flush=True)
    print(f"backend: {args.backend}", flush=True)

    source = make_frame_source(args)
    source.open()

    try:
        while not stop_event.is_set():
            frame = source.read()
            if frame is None:
                print("warning: could not read frame", flush=True)
                time.sleep(0.05)
                continue

            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
            )
            if ok:
                shared_frame.update(encoded.tobytes())

    finally:
        source.close()
        server.shutdown()
        server.server_close()
        print("wide-angle camera stream stopped", flush=True)


if __name__ == "__main__":
    main()
