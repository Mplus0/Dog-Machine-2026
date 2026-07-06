#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import re


DEFAULT_TOPIC_SPECS = [
    "color=/camera/color/image_raw:bgr8",
    "depth=/camera/depth/image_rect_raw:passthrough",
]


def safe_name(text):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_") or "topic"


def parse_topic_spec(spec):
    if "=" not in spec:
        raise ValueError("topic spec must be label=/topic[:encoding]: %s" % spec)
    label, rest = spec.split("=", 1)
    label = safe_name(label)
    if ":" in rest:
        topic, encoding = rest.rsplit(":", 1)
    else:
        topic, encoding = rest, "passthrough"
    topic = topic.strip()
    encoding = encoding.strip() or "passthrough"
    if not label or not topic:
        raise ValueError("invalid topic spec: %s" % spec)
    return label, topic, encoding


def read_stamp_file(path):
    values = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                values[float(parts[0])] = parts[1:]
    return values


def associate(first, second, max_difference):
    first_keys = list(first.keys())
    second_keys = list(second.keys())
    candidates = [
        (abs(a - b), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - b) <= max_difference
    ]
    candidates.sort()
    matches = []
    for _, a, b in candidates:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract image topics from a ROS bag into timestamped image files."
    )
    parser.add_argument("--bag", required=True, help="Input .bag file.")
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Output directory. One subdirectory and one timestamp txt file are created per label.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=None,
        help=(
            "Topic spec label=/topic[:encoding]. Can be repeated. "
            "Defaults to D435i color/depth topics."
        ),
    )
    parser.add_argument(
        "--associate",
        nargs=2,
        metavar=("FIRST_LABEL", "SECOND_LABEL"),
        help="Write associate_FIRST_SECOND.txt for two extracted labels.",
    )
    parser.add_argument(
        "--max-difference",
        type=float,
        default=0.03,
        help="Max timestamp difference for association, seconds.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Max frames per topic. 0 means unlimited.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality for non-depth images.",
    )
    return parser.parse_args()


def choose_extension(label, encoding):
    text = ("%s %s" % (label, encoding)).lower()
    if "depth" in text or encoding == "passthrough":
        return ".png"
    return ".jpg"


def write_association(output_dir, labels, max_difference):
    first_label, second_label = labels
    first_path = os.path.join(output_dir, first_label + ".txt")
    second_path = os.path.join(output_dir, second_label + ".txt")
    first = read_stamp_file(first_path)
    second = read_stamp_file(second_path)
    matches = associate(first, second, max_difference)

    assoc_name = "associate_%s_%s.txt" % (first_label, second_label)
    assoc_path = os.path.join(output_dir, assoc_name)
    with open(assoc_path, "w", encoding="utf-8") as handle:
        for a, b in matches:
            handle.write(
                "%.6f %s %.6f %s\n"
                % (a, " ".join(first[a]), b, " ".join(second[b]))
            )
    print("association written: %s, matches=%d" % (assoc_path, len(matches)), flush=True)


def main():
    args = parse_args()

    import cv2
    import rosbag
    from cv_bridge import CvBridge

    topic_specs = args.topic or DEFAULT_TOPIC_SPECS
    entries = [parse_topic_spec(spec) for spec in topic_specs]
    topic_to_entry = {topic: (label, encoding) for label, topic, encoding in entries}

    os.makedirs(args.output_dir, exist_ok=True)
    for label, _, _ in entries:
        os.makedirs(os.path.join(args.output_dir, label), exist_ok=True)

    bridge = CvBridge()
    counts = {label: 0 for label, _, _ in entries}
    txt_handles = {
        label: open(os.path.join(args.output_dir, label + ".txt"), "w", encoding="utf-8")
        for label, _, _ in entries
    }

    print("extracting bag: %s" % args.bag, flush=True)
    for label, topic, encoding in entries:
        print("  %s <- %s (%s)" % (label, topic, encoding), flush=True)

    try:
        with rosbag.Bag(args.bag, "r") as bag:
            for topic, msg, _ in bag.read_messages(topics=list(topic_to_entry.keys())):
                label, encoding = topic_to_entry[topic]
                if args.limit > 0 and counts[label] >= args.limit:
                    continue

                stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
                image = bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
                ext = choose_extension(label, encoding)
                filename = "%.6f%s" % (stamp, ext)
                rel_path = os.path.join(label, filename)
                out_path = os.path.join(args.output_dir, rel_path)

                if ext == ".jpg":
                    cv2.imwrite(
                        out_path,
                        image,
                        [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(args.jpeg_quality, 100))],
                    )
                else:
                    cv2.imwrite(out_path, image)

                txt_handles[label].write("%.6f %s\n" % (stamp, rel_path.replace("\\", "/")))
                counts[label] += 1
    finally:
        for handle in txt_handles.values():
            handle.close()

    for label, count in sorted(counts.items()):
        print("%s frames: %d" % (label, count), flush=True)

    if args.associate:
        write_association(args.output_dir, args.associate, args.max_difference)


if __name__ == "__main__":
    main()
