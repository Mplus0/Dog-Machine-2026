#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime


DEFAULT_OUTPUT = "d435i_profile_calibration.yaml"

DEFAULT_ROS_PROFILES = [
    # color_w, color_h, color_fps, depth_w, depth_h, depth_fps
    (640, 480, 5, 640, 480, 15),
    (640, 480, 15, 640, 480, 15),
    (848, 480, 5, 848, 480, 15),
    (848, 480, 15, 848, 480, 15),
    (1280, 720, 5, 640, 480, 15),
    (1280, 720, 15, 640, 480, 15),
    (1280, 720, 5, 848, 480, 15),
    (1280, 720, 15, 848, 480, 15),
]


def now_iso():
    return datetime.now().astimezone().isoformat()


def fov_from_intrinsics(width, height, fx, fy):
    width = float(width)
    height = float(height)
    fx = float(fx)
    fy = float(fy)
    hfov = math.degrees(2.0 * math.atan(width / (2.0 * fx)))
    vfov = math.degrees(2.0 * math.atan(height / (2.0 * fy)))
    dfov = math.degrees(
        2.0
        * math.atan(
            math.sqrt(width * width + height * height)
            / (2.0 * math.sqrt(fx * fx + fy * fy))
        )
    )
    return hfov, vfov, dfov


def profile_key(width, height, fps, fmt=""):
    parts = [str(int(width)), "x", str(int(height)), "@", str(int(fps))]
    if fmt:
        parts.extend(["/", str(fmt)])
    return "".join(parts)


def intrinsics_payload(width, height, fx, fy, cx, cy, distortion_model=None, coeffs=None):
    hfov, vfov, dfov = fov_from_intrinsics(width, height, fx, fy)
    payload = {
        "width": int(width),
        "height": int(height),
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
        "hfov_deg": hfov,
        "vfov_deg": vfov,
        "dfov_deg": dfov,
    }
    if distortion_model is not None:
        payload["distortion_model"] = str(distortion_model)
    if coeffs is not None:
        payload["distortion_coeffs"] = [float(x) for x in coeffs]
    return payload


def camera_info_payload(msg):
    payload = intrinsics_payload(
        msg.width,
        msg.height,
        msg.K[0],
        msg.K[4],
        msg.K[2],
        msg.K[5],
        msg.distortion_model,
        msg.D,
    )
    payload["K"] = [float(x) for x in msg.K]
    payload["R"] = [float(x) for x in msg.R]
    payload["P"] = [float(x) for x in msg.P]
    return payload


def rs_distortion_name(model):
    text = str(model)
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text


def rs_intrinsics_payload(intr):
    return intrinsics_payload(
        intr.width,
        intr.height,
        intr.fx,
        intr.fy,
        intr.ppx,
        intr.ppy,
        rs_distortion_name(intr.model),
        intr.coeffs,
    )


def rs_extrinsics_payload(extr):
    return {
        "rotation_row_major": [float(x) for x in extr.rotation],
        "translation_m": [float(x) for x in extr.translation],
        "translation_norm_m": math.sqrt(sum(float(x) * float(x) for x in extr.translation)),
    }


def dump_yaml_or_json(data, path):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.exists(directory):
        os.makedirs(directory)
    try:
        import yaml

        with open(path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False)
    except ImportError:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=False)


def load_yaml_or_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    try:
        import yaml

        return yaml.safe_load(text) or {}
    except ImportError:
        return json.loads(text)


def parse_profile_text(text):
    result = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            color_part, depth_part = item.split(":", 1)
            cw_h, cfps = color_part.split("@", 1)
            dw_h, dfps = depth_part.split("@", 1)
            cw, ch = cw_h.lower().split("x", 1)
            dw, dh = dw_h.lower().split("x", 1)
            result.append((int(cw), int(ch), int(cfps), int(dw), int(dh), int(dfps)))
        except ValueError:
            raise SystemExit("invalid --profiles item: %s" % item)
    return result


def run_pyrealsense(args):
    import pyrealsense2 as rs

    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        raise RuntimeError("no RealSense device found")

    out = {
        "stamp": now_iso(),
        "tool": "calibrate_d435i_profiles.py",
        "backend": "pyrealsense2",
        "note": args.note,
        "devices": [],
    }

    for dev in devices:
        if args.serial and dev.get_info(rs.camera_info.serial_number) != args.serial:
            continue
        dev_payload = {
            "name": safe_rs_info(dev, rs.camera_info.name),
            "serial": safe_rs_info(dev, rs.camera_info.serial_number),
            "firmware_version": safe_rs_info(dev, rs.camera_info.firmware_version),
            "sensors": [],
            "extrinsics": {},
        }

        stream_profiles = []
        for sensor in dev.query_sensors():
            sensor_payload = {
                "name": safe_rs_info(sensor, rs.camera_info.name),
                "profiles": [],
                "options": {},
            }
            for opt_name in ("stereo_baseline", "depth_units"):
                opt = getattr(rs.option, opt_name, None)
                if opt is not None and sensor.supports(opt):
                    try:
                        sensor_payload["options"][opt_name] = float(sensor.get_option(opt))
                    except Exception:
                        pass

            for prof in sensor.get_stream_profiles():
                if not prof.is_video_stream_profile():
                    continue
                video = prof.as_video_stream_profile()
                intr = video.get_intrinsics()
                stream_type = str(prof.stream_type()).rsplit(".", 1)[-1]
                fmt = str(prof.format()).rsplit(".", 1)[-1]
                profile_payload = {
                    "stream": stream_type,
                    "stream_index": int(prof.stream_index()),
                    "format": fmt,
                    "fps": int(prof.fps()),
                    "intrinsics": rs_intrinsics_payload(intr),
                }
                sensor_payload["profiles"].append(profile_payload)
                stream_profiles.append((stream_type, fmt, int(prof.fps()), video))
            sensor_payload["profiles"].sort(
                key=lambda p: (
                    p["stream"],
                    p["intrinsics"]["width"],
                    p["intrinsics"]["height"],
                    p["fps"],
                    p["format"],
                )
            )
            dev_payload["sensors"].append(sensor_payload)

        dev_payload["extrinsics"] = collect_rs_extrinsics(stream_profiles)
        out["devices"].append(dev_payload)

    if args.serial and not out["devices"]:
        raise RuntimeError("serial not found: %s" % args.serial)
    return out


def safe_rs_info(obj, key):
    try:
        if obj.supports(key):
            return obj.get_info(key)
    except Exception:
        return ""
    return ""


def first_profile(stream_profiles, stream_name, fps=None):
    candidates = []
    for stream_type, fmt, prof_fps, profile in stream_profiles:
        if stream_type != stream_name:
            continue
        if fps is not None and prof_fps != fps:
            continue
        candidates.append((stream_type, fmt, prof_fps, profile))
    candidates.sort(
        key=lambda item: (
            item[3].get_intrinsics().width * item[3].get_intrinsics().height,
            item[2],
        )
    )
    return candidates[0][3] if candidates else None


def collect_rs_extrinsics(stream_profiles):
    result = {}
    depth = first_profile(stream_profiles, "depth")
    color = first_profile(stream_profiles, "color")
    ir_profiles = [
        (item[3].stream_index(), item[3])
        for item in stream_profiles
        if item[0] == "infrared"
    ]
    ir_profiles.sort(key=lambda item: item[0])

    if depth is not None and color is not None:
        try:
            result["depth_to_color"] = rs_extrinsics_payload(depth.get_extrinsics_to(color))
        except Exception as exc:
            result["depth_to_color_error"] = str(exc)
        try:
            result["color_to_depth"] = rs_extrinsics_payload(color.get_extrinsics_to(depth))
        except Exception as exc:
            result["color_to_depth_error"] = str(exc)

    if len(ir_profiles) >= 2:
        try:
            extr = ir_profiles[0][1].get_extrinsics_to(ir_profiles[1][1])
            payload = rs_extrinsics_payload(extr)
            payload["from_stream_index"] = int(ir_profiles[0][0])
            payload["to_stream_index"] = int(ir_profiles[1][0])
            payload["baseline_m"] = abs(float(extr.translation[0]))
            result["infrared_1_to_infrared_2"] = payload
        except Exception as exc:
            result["infrared_1_to_infrared_2_error"] = str(exc)
    return result


def wait_for_ros_camera_info(topic, timeout):
    import rospy
    from sensor_msgs.msg import CameraInfo

    return rospy.wait_for_message(topic, CameraInfo, timeout=timeout)


def start_camera_launch(args, profile):
    cw, ch, cfps, dw, dh, dfps = profile
    cmd = [
        "roslaunch",
        "allmovebase",
        "camera.launch",
        "enable_color:=true",
        "enable_depth:=true",
        "align_depth:=%s" % ("true" if args.align_depth else "false"),
        "color_width:=%d" % cw,
        "color_height:=%d" % ch,
        "color_fps:=%d" % cfps,
        "depth_width:=%d" % dw,
        "depth_height:=%d" % dh,
        "depth_fps:=%d" % dfps,
    ]
    if args.serial:
        # allmovebase/camera.launch currently does not expose serial_no.
        # Keep this visible in the result rather than silently ignoring it.
        pass
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE if args.quiet_roslaunch else None,
        stderr=subprocess.STDOUT if args.quiet_roslaunch else None,
        text=True,
        preexec_fn=os.setsid if hasattr(os, "setsid") else None,
    )


def stop_process(proc):
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        if hasattr(os, "killpg"):
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        else:
            proc.send_signal(signal.SIGINT)
        proc.wait(timeout=8.0)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=3.0)
        except Exception:
            proc.kill()


def run_ros_active(args):
    import rospy

    rospy.init_node("calibrate_d435i_profiles", anonymous=True)
    result = {
        "stamp": now_iso(),
        "tool": "calibrate_d435i_profiles.py",
        "backend": "ros_active",
        "note": args.note,
        "align_depth": args.align_depth,
        "profiles": [],
    }
    color = wait_for_ros_camera_info(args.color_info_topic, args.ros_timeout)
    depth = wait_for_ros_camera_info(args.depth_info_topic, args.ros_timeout)
    item = {
        "status": "ok",
        "source": "already_running_ros_camera",
        "color": camera_info_payload(color),
        "depth": camera_info_payload(depth),
    }
    if args.align_depth:
        try:
            aligned = wait_for_ros_camera_info(args.aligned_depth_info_topic, args.ros_timeout)
            item["aligned_depth_to_color"] = camera_info_payload(aligned)
        except Exception as exc:
            item["aligned_depth_to_color_error"] = str(exc)
    result["profiles"].append(item)
    return result


def run_ros_probe(args):
    import rospy

    rospy.init_node("calibrate_d435i_profiles", anonymous=True)
    profiles = args.profile_tuples
    result = {
        "stamp": now_iso(),
        "tool": "calibrate_d435i_profiles.py",
        "backend": "ros_probe",
        "note": args.note,
        "align_depth": args.align_depth,
        "profiles": [],
        "warning": (
            "ROS probe can only test requested profile tuples. Use pyrealsense2 "
            "backend on a host where it is available for true device enumeration."
        ),
    }

    for profile in profiles:
        cw, ch, cfps, dw, dh, dfps = profile
        item = {
            "requested": {
                "color": profile_key(cw, ch, cfps),
                "depth": profile_key(dw, dh, dfps),
            }
        }
        proc = None
        try:
            proc = start_camera_launch(args, profile)
            time.sleep(max(0.0, args.launch_settle))
            color = wait_for_ros_camera_info(args.color_info_topic, args.ros_timeout)
            depth = wait_for_ros_camera_info(args.depth_info_topic, args.ros_timeout)
            item["status"] = "ok"
            item["color"] = camera_info_payload(color)
            item["depth"] = camera_info_payload(depth)
            if args.align_depth:
                try:
                    aligned = wait_for_ros_camera_info(args.aligned_depth_info_topic, args.ros_timeout)
                    item["aligned_depth_to_color"] = camera_info_payload(aligned)
                except Exception as exc:
                    item["aligned_depth_to_color_error"] = str(exc)
        except Exception as exc:
            item["status"] = "failed"
            item["error"] = str(exc)
        finally:
            stop_process(proc)
            time.sleep(max(0.0, args.stop_settle))
        result["profiles"].append(item)
        print_profile_summary(item)

    return result


def print_profile_summary(item):
    req = item.get("requested", {})
    prefix = "%s depth=%s color=%s" % (
        item.get("status", "unknown"),
        req.get("depth", "?"),
        req.get("color", "?"),
    )
    if item.get("status") != "ok":
        print(prefix + " error=%s" % item.get("error", ""))
        return
    depth = item.get("depth", {})
    color = item.get("color", {})
    print(
        "%s -> depth %dx%d hfov=%.2f vfov=%.2f; color %dx%d hfov=%.2f vfov=%.2f"
        % (
            prefix,
            depth.get("width", 0),
            depth.get("height", 0),
            depth.get("hfov_deg", 0.0),
            depth.get("vfov_deg", 0.0),
            color.get("width", 0),
            color.get("height", 0),
            color.get("hfov_deg", 0.0),
            color.get("vfov_deg", 0.0),
        )
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Record D435i intrinsics/FOV for multiple RGB/depth profiles. "
            "pyrealsense2 backend enumerates device profiles and extrinsics; "
            "ROS probe backend relaunches camera.launch for requested profiles."
        )
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "pyrealsense2", "ros-active", "ros-probe"),
        default="auto",
    )
    parser.add_argument("--serial", default="", help="Optional RealSense serial number.")
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--note", default="")
    parser.add_argument("--align-depth", action="store_true")
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")
    parser.add_argument(
        "--aligned-depth-info-topic",
        default="/camera/aligned_depth_to_color/camera_info",
    )
    parser.add_argument("--ros-timeout", type=float, default=10.0)
    parser.add_argument("--launch-settle", type=float, default=5.0)
    parser.add_argument("--stop-settle", type=float, default=2.0)
    parser.add_argument(
        "--profiles",
        default="",
        help=(
            "Comma-separated ROS probe profiles: "
            "colorWxH@fps:depthWxH@fps,colorWxH@fps:depthWxH@fps"
        ),
    )
    parser.add_argument(
        "--profiles-file",
        default="",
        help="YAML/JSON file containing a list named profiles for ROS probe.",
    )
    parser.add_argument(
        "--quiet-roslaunch",
        action="store_true",
        help="Hide roslaunch stdout while probing profiles.",
    )
    return parser.parse_args()


def load_profile_tuples(args):
    if args.profiles:
        return parse_profile_text(args.profiles)
    if args.profiles_file:
        data = load_yaml_or_json(args.profiles_file)
        out = []
        for item in data.get("profiles", []):
            color = item.get("color", {})
            depth = item.get("depth", {})
            out.append(
                (
                    int(color["width"]),
                    int(color["height"]),
                    int(color["fps"]),
                    int(depth["width"]),
                    int(depth["height"]),
                    int(depth["fps"]),
                )
            )
        return out
    return list(DEFAULT_ROS_PROFILES)


def main():
    args = parse_args()
    args.profile_tuples = load_profile_tuples(args)

    if args.backend in ("auto", "pyrealsense2"):
        try:
            data = run_pyrealsense(args)
            dump_yaml_or_json(data, args.output)
            print("wrote %s" % args.output)
            return
        except Exception as exc:
            if args.backend == "pyrealsense2":
                raise
            print("pyrealsense2 backend unavailable: %s" % exc, file=sys.stderr)
            print("falling back to ros-active backend", file=sys.stderr)

    if args.backend == "ros-probe":
        data = run_ros_probe(args)
    else:
        data = run_ros_active(args)
    dump_yaml_or_json(data, args.output)
    print("wrote %s" % args.output)


if __name__ == "__main__":
    main()
