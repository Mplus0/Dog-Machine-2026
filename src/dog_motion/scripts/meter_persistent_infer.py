#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
import json
import sys
from collections import Counter

import cv2
from ultralytics import YOLO


def detect_one(model, image_path, min_confidence):
    image = cv2.imread(image_path)
    if image is None:
        return None

    results = model(image, verbose=False)
    classes = []
    for result in results:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue
        conf_values = getattr(boxes, "conf", [])
        for index, cls_value in enumerate(boxes.cls):
            if conf_values is not None and len(conf_values) > index:
                if float(conf_values[index]) < min_confidence:
                    continue
            classes.append(str(model.names[int(cls_value)]))

    regions = [name for name in classes if name in ("A", "B", "C", "D")]
    statuses = [name for name in classes if name in ("low", "normal", "high")]
    if not regions or not statuses:
        return None
    return regions[0], statuses[0]


def infer_dir(model, image_dir, min_confidence):
    image_paths = sorted(glob.glob(image_dir.rstrip("/") + "/*.jpg"))
    if not image_paths:
        return None, "no images"

    votes = []
    for path in image_paths:
        result = detect_one(model, path, min_confidence)
        if result:
            votes.append(result)
            print("SAMPLE:%s:%s,%s" % (path, result[0], result[1]), flush=True)
        else:
            print("SAMPLE:%s:none" % path, flush=True)

    if not votes:
        return None, "no valid result"

    region = Counter(item[0] for item in votes).most_common(1)[0][0]
    status = Counter(item[1] for item in votes).most_common(1)[0][0]
    return (region, status), None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    args = parser.parse_args()

    model = YOLO(args.model)
    print("READY", flush=True)

    for raw_line in sys.stdin:
        raw_line = raw_line.strip()
        if not raw_line:
            continue
        if raw_line == "QUIT":
            print("BYE", flush=True)
            return 0

        try:
            job = json.loads(raw_line)
            trigger = str(job.get("trigger", "")).strip()
            image_dir = str(job["images"]).strip()
            min_confidence = float(job.get("min_confidence", args.min_confidence))
        except Exception as exc:
            print("ERROR:bad job:%s" % exc, flush=True)
            continue

        result, error = infer_dir(model, image_dir, min_confidence)
        if error:
            print("ERROR:%s:%s" % (trigger, error), flush=True)
            continue

        print("RESULT:%s,%s,%s" % (trigger, result[0], result[1]), flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
