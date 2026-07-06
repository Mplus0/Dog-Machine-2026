# -*- coding: utf-8 -*-

import argparse
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


DEFAULT_CAMERA_DEVICE = "/dev/realsense_rgb"


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


def open_camera(camera_source):
    describe_camera_source(camera_source)

    if isinstance(camera_source, str) and not os.path.exists(camera_source):
        raise RuntimeError(
            f"Could not find camera path: {camera_source}. "
            f"Try --camera-index 0 or --camera /dev/videoX."
        )

    print(f"trying camera: {camera_source}", flush=True)

    cap = cv2.VideoCapture(camera_source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(
            f"Could not open camera: {camera_source}. "
            f"Try --camera-index 0 or --camera /dev/videoX."
        )

    ret, frame = cap.read()
    if not ret or frame is None:
        cap.release()
        raise RuntimeError(f"Camera opened but could not read frame: {camera_source}.")

    print(f"camera opened: {camera_source}", flush=True)
    print(f"first frame shape: {frame.shape}", flush=True)

    return cap


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
  <title>Camera Stream</title>
  <style>
    body {
      margin: 0;
      background: #111;
      color: #eee;
      font-family: Arial, sans-serif;
    }
    main {
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 16px;
      box-sizing: border-box;
    }
    img {
      width: min(100%, 960px);
      height: auto;
      background: #000;
    }
  </style>
</head>
<body>
  <main>
    <img src="/stream.mjpg" alt="camera stream">
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
        description="Capture camera frames and stream them through an MJPEG web server."
    )

    parser.add_argument(
        "camera_index_pos",
        nargs="?",
        type=int,
        help="Optional camera index, for example: python3 camera_stream.py 0",
    )

    parser.add_argument(
        "--camera-index",
        type=int,
        default=None,
        help="Camera index, for example 0 means /dev/video0.",
    )

    parser.add_argument(
        "--camera",
        default=DEFAULT_CAMERA_DEVICE,
        help="Camera device path, for example /dev/realsense_rgb or /dev/video4.",
    )

    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="HTTP bind address.",
    )

    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="HTTP stream port.",
    )

    parser.add_argument(
        "--stream-fps",
        type=int,
        default=10,
        help="MJPEG stream FPS.",
    )

    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=70,
        help="JPEG quality, 1-100.",
    )

    return parser.parse_args()


def resolve_camera_source(args):
    if args.camera_index is not None:
        return args.camera_index

    if args.camera_index_pos is not None:
        return args.camera_index_pos

    return args.camera


def main():
    args = parse_args()

    camera_source = resolve_camera_source(args)
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
    print(f"camera source: {camera_source}", flush=True)
    print(f"open http://<device-ip>:{args.port} from your PC browser", flush=True)

    cap = open_camera(camera_source)

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()

            if not ret or frame is None:
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
        cap.release()
        server.shutdown()
        server.server_close()
        print("camera stream stopped", flush=True)


if __name__ == "__main__":
    main()