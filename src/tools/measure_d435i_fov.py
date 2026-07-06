#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys


def fov_from_intrinsics(width, height, fx, fy):
    hfov = math.degrees(2.0 * math.atan(float(width) / (2.0 * float(fx))))
    vfov = math.degrees(2.0 * math.atan(float(height) / (2.0 * float(fy))))
    dfov = math.degrees(
        2.0
        * math.atan(
            math.sqrt(float(width) ** 2 + float(height) ** 2)
            / (2.0 * math.sqrt(float(fx) ** 2 + float(fy) ** 2))
        )
    )
    return hfov, vfov, dfov


def print_camera_result(name, width, height, fx, fy, cx, cy, source):
    hfov, vfov, dfov = fov_from_intrinsics(width, height, fx, fy)
    print("[%s]" % name)
    print("source: %s" % source)
    print("resolution: %dx%d" % (int(width), int(height)))
    print("fx: %.6f" % float(fx))
    print("fy: %.6f" % float(fy))
    print("cx: %.6f" % float(cx))
    print("cy: %.6f" % float(cy))
    print("hfov_deg: %.6f" % hfov)
    print("vfov_deg: %.6f" % vfov)
    print("dfov_deg: %.6f" % dfov)
    print("")


def measure_realsense(args):
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)
    if args.enable_rgb:
        config.enable_stream(
            rs.stream.color,
            int(args.rgb_width),
            int(args.rgb_height),
            rs.format.bgr8,
            int(args.rgb_fps),
        )
    if args.enable_depth:
        config.enable_stream(
            rs.stream.depth,
            int(args.depth_width),
            int(args.depth_height),
            rs.format.z16,
            int(args.depth_fps),
        )

    profile = pipeline.start(config)
    try:
        # Wait for a few frames so RealSense applies the actually selected profiles.
        for _ in range(max(1, args.warmup_frames)):
            pipeline.wait_for_frames(timeout_ms=5000)

        if args.enable_rgb:
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_profile.get_intrinsics()
            print_camera_result(
                "d435i_color",
                intr.width,
                intr.height,
                intr.fx,
                intr.fy,
                intr.ppx,
                intr.ppy,
                "pyrealsense2",
            )
        if args.enable_depth:
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            intr = depth_profile.get_intrinsics()
            print_camera_result(
                "d435i_depth",
                intr.width,
                intr.height,
                intr.fx,
                intr.fy,
                intr.ppx,
                intr.ppy,
                "pyrealsense2",
            )
    finally:
        pipeline.stop()


def wait_ros_camera_info(topic, label, timeout):
    import rospy
    from sensor_msgs.msg import CameraInfo

    msg = rospy.wait_for_message(topic, CameraInfo, timeout=timeout)
    print_camera_result(
        label,
        msg.width,
        msg.height,
        msg.K[0],
        msg.K[4],
        msg.K[2],
        msg.K[5],
        "ros:%s" % topic,
    )


def measure_ros(args):
    import rospy

    rospy.init_node("measure_d435i_fov", anonymous=True)
    if args.enable_rgb:
        wait_ros_camera_info(args.rgb_info_topic, "d435i_color", args.ros_timeout)
    if args.enable_depth:
        wait_ros_camera_info(args.depth_info_topic, "d435i_depth", args.ros_timeout)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Measure current D435i RGB/depth equivalent FOV from live intrinsics. "
            "FOV is computed from width, height, fx and fy, so it follows the active resolution/profile."
        )
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "realsense", "ros"),
        default="ros",
        help="Frame source. Default reads ROS CameraInfo because pyrealsense2 is not available on the robot.",
    )
    parser.add_argument("--serial", default="", help="Optional RealSense serial number.")
    parser.add_argument("--rgb-width", type=int, default=640)
    parser.add_argument("--rgb-height", type=int, default=480)
    parser.add_argument("--rgb-fps", type=int, default=15)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--rgb-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--ros-timeout", type=float, default=10.0)
    parser.add_argument("--warmup-frames", type=int, default=5)
    parser.add_argument("--rgb-only", action="store_true", help="Measure RGB/color only.")
    parser.add_argument("--depth-only", action="store_true", help="Measure depth only.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rgb_only and args.depth_only:
        raise SystemExit("--rgb-only and --depth-only cannot be used together")
    args.enable_rgb = not args.depth_only
    args.enable_depth = not args.rgb_only

    if args.backend == "auto":
        try:
            measure_ros(args)
            return
        except Exception as exc:
            print("ROS CameraInfo backend unavailable: %s" % exc, file=sys.stderr)
            print("falling back to pyrealsense2 backend", file=sys.stderr)

    if args.backend == "realsense":
        try:
            measure_realsense(args)
            return
        except Exception as exc:
            if args.backend == "realsense":
                raise
            print("pyrealsense2 backend unavailable: %s" % exc, file=sys.stderr)
            print("falling back to ROS CameraInfo backend", file=sys.stderr)

    measure_ros(args)


if __name__ == "__main__":
    main()
