#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import glob
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--images", required=True)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    args = parser.parse_args()

    image_paths = sorted(glob.glob(args.images.rstrip("/") + "/*.jpg"))
    if not image_paths:
        print("ERROR:no images", file=sys.stderr)
        return 2

    model = YOLO(args.model)
    votes = []
    for path in image_paths:
        result = detect_one(model, path, args.min_confidence)
        if result:
            votes.append(result)
            print("SAMPLE:%s:%s,%s" % (path, result[0], result[1]))
        else:
            print("SAMPLE:%s:none" % path)

    if not votes:
        print("ERROR:no valid result", file=sys.stderr)
        return 3

    region = Counter(item[0] for item in votes).most_common(1)[0][0]
    status = Counter(item[1] for item in votes).most_common(1)[0][0]
    print("RESULT:%s,%s" % (region, status))
    return 0


if __name__ == "__main__":
    sys.exit(main())
