#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import datetime
import math
import os
import random
import time

import numpy as np
import yaml


DEFAULT_OUTPUT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "d435i_ground_tilt_calibration.yaml",
)


def quaternion_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm <= 1.0e-12:
        return [0.0, 0.0, 0.0, 1.0]
    axis = axis / norm
    half = 0.5 * angle
    s = math.sin(half)
    return [float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(half))]


def rotation_between_vectors(src, dst):
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    src = src / max(np.linalg.norm(src), 1.0e-12)
    dst = dst / max(np.linalg.norm(dst), 1.0e-12)

    dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
    if dot > 1.0 - 1.0e-9:
        return [0.0, 0.0, 0.0, 1.0], 0.0
    if dot < -1.0 + 1.0e-9:
        axis = np.cross(src, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1.0e-6:
            axis = np.cross(src, [0.0, 0.0, 1.0])
        return quaternion_from_axis_angle(axis, math.pi), math.pi

    axis = np.cross(src, dst)
    angle = math.acos(dot)
    return quaternion_from_axis_angle(axis, angle), angle


def fit_plane_svd(points):
    centroid = np.mean(points, axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1, :]
    normal = normal / max(np.linalg.norm(normal), 1.0e-12)
    return normal, centroid


def point_plane_distances(points, normal, point_on_plane):
    return np.abs(np.dot(points - point_on_plane, normal))


def robust_fit_plane(points, iterations, sample_size, inlier_threshold, min_inliers):
    if len(points) < max(sample_size, min_inliers):
        raise RuntimeError("not enough points for plane fit: %d" % len(points))

    best_inlier_idx = None
    best_score = -1
    count = len(points)

    for _ in range(iterations):
        sample_idx = random.sample(range(count), sample_size)
        sample = points[sample_idx, :]
        normal, centroid = fit_plane_svd(sample)
        distances = point_plane_distances(points, normal, centroid)
        inlier_idx = np.nonzero(distances < inlier_threshold)[0]
        score = len(inlier_idx)
        if score > best_score:
            best_score = score
            best_inlier_idx = inlier_idx

    if best_inlier_idx is None or len(best_inlier_idx) < min_inliers:
        raise RuntimeError(
            "plane fit failed, inliers=%s min_inliers=%d"
            % (0 if best_inlier_idx is None else len(best_inlier_idx), min_inliers)
        )

    normal, centroid = fit_plane_svd(points[best_inlier_idx, :])
    return normal, centroid, points[best_inlier_idx, :]


def sample_depth_points(depth, fx, fy, cx, cy, args):
    height, width = depth.shape[:2]
    u0 = int(width * args.roi_left)
    u1 = int(width * args.roi_right)
    v0 = int(height * args.roi_top)
    v1 = int(height * args.roi_bottom)
    u0 = max(0, min(width - 1, u0))
    u1 = max(u0 + 1, min(width, u1))
    v0 = max(0, min(height - 1, v0))
    v1 = max(v0 + 1, min(height, v1))

    step = max(1, int(args.pixel_step))
    roi = depth[v0:v1:step, u0:u1:step]
    vv, uu = np.mgrid[v0:v1:step, u0:u1:step]

    z = roi.astype(np.float64)
    if args.depth_scale > 0.0:
        z = z * args.depth_scale

    valid = np.isfinite(z)
    valid &= z >= args.min_depth
    valid &= z <= args.max_depth
    valid &= z > 0.0

    if args.reject_top_percent > 0.0:
        cutoff = np.percentile(z[valid], 100.0 - args.reject_top_percent) if np.any(valid) else 0.0
        valid &= z <= cutoff

    z = z[valid]
    u = uu[valid].astype(np.float64)
    v = vv[valid].astype(np.float64)

    if len(z) > args.max_points:
        idx = np.random.choice(len(z), args.max_points, replace=False)
        z = z[idx]
        u = u[idx]
        v = v[idx]

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.column_stack((x, y, z)), (u0, v0, u1, v1)


def summarize_normal(normal):
    # ROS optical frame convention: +x right, +y down, +z forward.
    # The upward ground normal should be close to [0, -1, 0] when the depth
    # optical frame is level relative to the floor.
    if normal[1] > 0.0:
        normal = -normal
    expected_up = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    quat, angle = rotation_between_vectors(normal, expected_up)

    pitch_x = math.degrees(math.atan2(float(normal[2]), float(-normal[1])))
    roll_z = math.degrees(math.atan2(float(normal[0]), float(-normal[1])))
    total = math.degrees(angle)
    return normal, pitch_x, roll_z, total, quat


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Estimate D435i ground tilt by fitting a plane from depth image points. "
            "Run while the robot is standing still on a flat floor."
        )
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "realsense", "ros"),
        default="auto",
        help="Frame source. auto tries pyrealsense2 first, then ROS topics.",
    )
    parser.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
    parser.add_argument("--camera-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--width", type=int, default=640, help="RealSense depth width.")
    parser.add_argument("--height", type=int, default=480, help="RealSense depth height.")
    parser.add_argument("--fps", type=int, default=15, help="RealSense depth FPS.")
    parser.add_argument("--serial", default="", help="Optional RealSense serial number.")
    parser.add_argument("--samples", type=int, default=30, help="Number of depth frames to fit.")
    parser.add_argument("--settle-sec", type=float, default=1.0, help="Delay before sampling.")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=30,
        help="Depth frames to read and discard before sampling. Useful for bad D435i startup frames.",
    )
    parser.add_argument("--roi-left", type=float, default=0.15)
    parser.add_argument("--roi-right", type=float, default=0.85)
    parser.add_argument("--roi-top", type=float, default=0.45)
    parser.add_argument("--roi-bottom", type=float, default=0.95)
    parser.add_argument("--pixel-step", type=int, default=8)
    parser.add_argument("--min-depth", type=float, default=0.25)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument(
        "--depth-scale",
        type=float,
        default=0.0,
        help="Depth scale. 0 means infer: uint16 millimeters -> 0.001, float meters -> 1.0.",
    )
    parser.add_argument("--max-points", type=int, default=8000)
    parser.add_argument("--ransac-iterations", type=int, default=80)
    parser.add_argument("--ransac-sample-size", type=int, default=80)
    parser.add_argument("--inlier-threshold", type=float, default=0.025)
    parser.add_argument("--min-inliers", type=int, default=400)
    parser.add_argument(
        "--reject-top-percent",
        type=float,
        default=0.0,
        help="Reject farthest depth percent inside ROI before fitting. Usually leave 0.",
    )
    parser.add_argument(
        "--output-yaml",
        default=DEFAULT_OUTPUT_YAML,
        help="Calibration result YAML path.",
    )
    parser.add_argument(
        "--note",
        default="",
        help="Optional note saved with this measurement, for example bracket_v2.",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Print only, do not update output YAML.",
    )
    return parser.parse_args()


class RealSenseDepthSource:
    def __init__(self, args):
        self.args = args
        self.pipeline = None
        self.depth_scale = None
        self.intrinsics = None
        self.rs = None

    def open(self):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.args.serial:
            config.enable_device(self.args.serial)
        config.enable_stream(
            rs.stream.depth,
            int(self.args.width),
            int(self.args.height),
            rs.format.z16,
            int(self.args.fps),
        )
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())
        stream_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.intrinsics = stream_profile.get_intrinsics()
        print(
            "RealSense depth opened: %dx%d@%d scale=%.8f"
            % (self.args.width, self.args.height, self.args.fps, self.depth_scale),
            flush=True,
        )

    def camera_model(self):
        return (
            float(self.intrinsics.fx),
            float(self.intrinsics.fy),
            float(self.intrinsics.ppx),
            float(self.intrinsics.ppy),
        )

    def read_depth(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        return np.asanyarray(depth_frame.get_data())

    def close(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


class RosDepthSource:
    def __init__(self, args):
        self.args = args
        self.bridge = None
        self.rospy = None

    def open(self):
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image

        self.rospy = rospy
        self.Image = Image
        self.bridge = CvBridge()
        rospy.init_node("calibrate_d435i_ground_tilt", anonymous=False)
        rospy.loginfo("waiting for camera info: %s", self.args.camera_info_topic)
        self.info = rospy.wait_for_message(
            self.args.camera_info_topic,
            CameraInfo,
            timeout=10.0,
        )

    def camera_model(self):
        info = self.info
        return float(info.K[0]), float(info.K[4]), float(info.K[2]), float(info.K[5])

    def read_depth(self):
        msg = self.rospy.wait_for_message(self.args.depth_topic, self.Image, timeout=5.0)
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def close(self):
        return


def make_source(args):
    if args.backend in ("auto", "realsense"):
        try:
            source = RealSenseDepthSource(args)
            source.open()
            return "realsense", source
        except Exception as exc:
            if args.backend == "realsense":
                raise
            print("RealSense backend unavailable: %s" % exc, flush=True)
            print("falling back to ROS topic backend", flush=True)

    source = RosDepthSource(args)
    source.open()
    return "ros", source


def infer_depth_scale(args, depth, source):
    if args.depth_scale > 0.0:
        return args.depth_scale
    if isinstance(source, RealSenseDepthSource):
        return source.depth_scale
    return 0.001 if str(depth.dtype).startswith("uint") else 1.0


def discard_warmup_frames(source, count):
    count = max(0, int(count))
    if count <= 0:
        return 0

    print("discarding %d warmup depth frames" % count, flush=True)
    discarded = 0
    for index in range(count):
        try:
            depth = source.read_depth()
        except Exception as exc:
            print(
                "warmup frame %02d/%02d failed: %s"
                % (index + 1, count, exc),
                flush=True,
            )
            continue
        if depth is None:
            print(
                "warmup frame %02d/%02d skipped: no depth frame"
                % (index + 1, count),
                flush=True,
            )
            continue
        discarded += 1
    print("warmup complete: discarded %d/%d frames" % (discarded, count), flush=True)
    return discarded


def run_calibration(args):
    backend, source = make_source(args)
    fx, fy, cx, cy = source.camera_model()
    print("backend: %s" % backend, flush=True)
    print("camera intrinsics fx=%.3f fy=%.3f cx=%.3f cy=%.3f" % (fx, fy, cx, cy), flush=True)

    if args.settle_sec > 0.0:
        print("settling for %.1fs; keep the robot still" % args.settle_sec, flush=True)
        time.sleep(args.settle_sec)

    results = []
    roi = None
    depth_scale = None
    warmup_discarded = 0

    try:
        warmup_discarded = discard_warmup_frames(source, args.warmup_frames)

        for index in range(args.samples):
            depth = source.read_depth()
            if depth is None:
                print("sample %02d/%02d skipped: no depth frame" % (index + 1, args.samples), flush=True)
                continue
            if depth_scale is None:
                depth_scale = infer_depth_scale(args, depth, source)
                args.depth_scale = depth_scale
                print("depth_scale: %.8f" % depth_scale, flush=True)

            points, roi = sample_depth_points(depth, fx, fy, cx, cy, args)
            if len(points) < args.min_inliers:
                print(
                    "sample %02d/%02d skipped: only %d valid points in ROI"
                    % (index + 1, args.samples, len(points)),
                    flush=True,
                )
                continue

            normal, centroid, inliers = robust_fit_plane(
                points,
                args.ransac_iterations,
                args.ransac_sample_size,
                args.inlier_threshold,
                args.min_inliers,
            )
            normal, pitch_x, roll_z, total, quat = summarize_normal(normal)
            results.append((normal, pitch_x, roll_z, total, quat, centroid, len(points), len(inliers)))
            print(
                "sample %02d/%02d: pitch_x=%+.2fdeg roll_z=%+.2fdeg total=%+.2fdeg inliers=%d/%d"
                % (index + 1, args.samples, pitch_x, roll_z, total, len(inliers), len(points)),
                flush=True,
            )
    finally:
        source.close()

    if not results:
        raise RuntimeError("no valid plane fit results")

    normals = np.asarray([item[0] for item in results], dtype=np.float64)
    avg_normal = np.mean(normals, axis=0)
    avg_normal = avg_normal / max(np.linalg.norm(avg_normal), 1.0e-12)
    avg_normal, pitch_x, roll_z, total, quat = summarize_normal(avg_normal)
    pitch_values = np.asarray([item[1] for item in results], dtype=np.float64)
    roll_values = np.asarray([item[2] for item in results], dtype=np.float64)
    total_values = np.asarray([item[3] for item in results], dtype=np.float64)

    print("", flush=True)
    print("=== D435i ground tilt estimate ===", flush=True)
    print("backend: %s" % backend, flush=True)
    if backend == "ros":
        print("depth_topic: %s" % args.depth_topic, flush=True)
        print("camera_info_topic: %s" % args.camera_info_topic, flush=True)
    else:
        print("realsense_depth: %dx%d@%d" % (args.width, args.height, args.fps), flush=True)
    if roi is not None:
        print("roi_pixels: left=%d top=%d right=%d bottom=%d" % roi, flush=True)
    print("valid_samples: %d/%d" % (len(results), args.samples), flush=True)
    print(
        "ground_up_normal_in_depth_optical: [%.6f, %.6f, %.6f]"
        % (avg_normal[0], avg_normal[1], avg_normal[2]),
        flush=True,
    )
    print(
        "pitch_about_optical_x_deg: %.3f  std=%.3f"
        % (pitch_x, float(np.std(pitch_values))),
        flush=True,
    )
    print(
        "roll_about_optical_z_deg: %.3f  std=%.3f"
        % (roll_z, float(np.std(roll_values))),
        flush=True,
    )
    print("total_ground_normal_tilt_deg: %.3f" % total, flush=True)
    print(
        "normal_alignment_quaternion_xyzw: [%.6f, %.6f, %.6f, %.6f]"
        % (quat[0], quat[1], quat[2], quat[3]),
        flush=True,
    )
    print("", flush=True)
    print("Interpretation:", flush=True)
    print("- Optical frame convention: +x right, +y down, +z forward.", flush=True)
    print("- Level ground should have upward normal close to [0, -1, 0].", flush=True)
    print("- Use the signs above as a measured tilt first; verify in RViz before editing camera2base_tf.yaml.", flush=True)
    print("- If std is large, clear the ROI floor area or narrow the ROI and rerun.", flush=True)

    return {
        "stamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "note": args.note,
        "backend": backend,
        "source": {
            "depth_topic": args.depth_topic if backend == "ros" else None,
            "camera_info_topic": args.camera_info_topic if backend == "ros" else None,
            "serial": args.serial or None,
            "width": args.width if backend == "realsense" else None,
            "height": args.height if backend == "realsense" else None,
            "fps": args.fps if backend == "realsense" else None,
        },
        "roi_fraction": {
            "left": args.roi_left,
            "right": args.roi_right,
            "top": args.roi_top,
            "bottom": args.roi_bottom,
        },
        "roi_pixels": {
            "left": int(roi[0]),
            "top": int(roi[1]),
            "right": int(roi[2]),
            "bottom": int(roi[3]),
        } if roi is not None else None,
        "depth_scale": float(depth_scale if depth_scale is not None else args.depth_scale),
        "warmup_frames_requested": int(max(0, args.warmup_frames)),
        "warmup_frames_discarded": int(warmup_discarded),
        "valid_samples": int(len(results)),
        "requested_samples": int(args.samples),
        "ground_up_normal_in_depth_optical": [float(v) for v in avg_normal],
        "pitch_about_optical_x_deg": float(pitch_x),
        "pitch_std_deg": float(np.std(pitch_values)),
        "roll_about_optical_z_deg": float(roll_z),
        "roll_std_deg": float(np.std(roll_values)),
        "total_ground_normal_tilt_deg": float(total),
        "total_std_deg": float(np.std(total_values)),
        "normal_alignment_quaternion_xyzw": [float(v) for v in quat],
        "fit": {
            "inlier_threshold_m": float(args.inlier_threshold),
            "min_inliers": int(args.min_inliers),
            "pixel_step": int(args.pixel_step),
            "min_depth_m": float(args.min_depth),
            "max_depth_m": float(args.max_depth),
        },
    }


def save_result_yaml(path, result):
    data = {}
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
    history = data.get("history", [])
    if not isinstance(history, list):
        history = []
    history.append(result)
    data = {
        "latest": result,
        "history": history,
    }
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    print("saved calibration YAML: %s" % path, flush=True)


def main():
    args = parse_args()
    result = run_calibration(args)
    if not args.no_save:
        save_result_yaml(args.output_yaml, result)


if __name__ == "__main__":
    main()
