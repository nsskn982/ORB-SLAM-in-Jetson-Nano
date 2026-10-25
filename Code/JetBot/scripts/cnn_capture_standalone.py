#!/usr/bin/env python3
"""
@file cnn_capture_standalone.py
@brief Standalone CSI camera capture for CNN dataset collection.
       Produces the same output structure as cnn_data_collect.py
       (images/ subfolder + data.csv) without requiring a running
       ROS master or any active ROS topics.
@author Srikanth Nanda
@date 2026-06-03
"""

import argparse
import csv
import os
import time

import cv2


def build_pipeline(sensor_id, capture_width, capture_height,
                   output_width, output_height, fps, flip):
    """@brief Build nvarguscamerasrc GStreamer pipeline string."""
    return (
        "nvarguscamerasrc sensor-id={sid} ! "
        "video/x-raw(memory:NVMM), width=(int){cw}, height=(int){ch}, "
        "format=(string)NV12, framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method={flip} ! "
        "video/x-raw, width=(int){ow}, height=(int){oh}, "
        "format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
    ).format(sid=sensor_id, cw=capture_width, ch=capture_height,
             fps=fps, flip=flip, ow=output_width, oh=output_height)


CSV_HEADER = [
    "timestamp", "image",
    "pose_x", "pose_y", "pose_z",
    "pose_qx", "pose_qy", "pose_qz", "pose_qw",
    "collision_prob",
]


def open_csv(path):
    """@brief Open (or append to) the dataset CSV."""
    write_header = not os.path.exists(path)
    f = open(path, "a", newline="")
    writer = csv.writer(f)
    if write_header:
        writer.writerow(CSV_HEADER)
    return f, writer


def capture(args):
    """@brief Main capture loop."""
    images_dir = os.path.join(args.output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    csv_file, csv_writer = open_csv(os.path.join(args.output_dir, "data.csv"))

    pipeline = build_pipeline(args.sensor_id, args.capture_width,
                               args.capture_height, args.width,
                               args.height, args.fps, args.flip)

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit(
            "[ERROR] Could not open camera.\n"
            "  Try: sudo systemctl restart nvargus-daemon"
        )

    print("[INFO] Camera opened: {w}x{h} @ {fps} fps".format(
        w=args.width, h=args.height, fps=args.fps))
    print("[INFO] Saving '{cls}' class -> {d}".format(
        cls=args.label, d=args.output_dir))
    print("[INFO] Starting in 3 seconds -- position the JetBot now ...")
    time.sleep(3)

    save_period = 1.0 / max(1e-6, args.save_rate)
    last_save = 0.0
    saved = 0
    start = time.time()

    try:
        while True:
            if args.duration > 0 and (time.time() - start) >= args.duration:
                print("[INFO] Duration reached.")
                break
            if args.max_frames > 0 and saved >= args.max_frames:
                print("[INFO] Max frames reached.")
                break

            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            now = time.time()
            if now - last_save < save_period:
                continue

            timestamp_ns = int(now * 1e9)
            filename = "frame_{ts}.png".format(ts=timestamp_ns)
            image_path = os.path.join(images_dir, filename)
            if not cv2.imwrite(image_path, frame):
                continue

            csv_writer.writerow([now, os.path.join("images", filename),
                                  "", "", "", "", "", "", "", ""])
            csv_file.flush()
            last_save = now
            saved += 1

            if saved % 50 == 0:
                print("[INFO] {n} frames saved  ({e:.1f}s elapsed)".format(
                    n=saved, e=time.time() - start))

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        cap.release()
        csv_file.close()
        print("[INFO] Done. {n} frames -> {d}".format(
            n=saved, d=args.output_dir))


def main():
    """@brief CLI entrypoint."""
    parser = argparse.ArgumentParser(
        description="Standalone CNN data capture -- no ROS master required.")
    parser.add_argument("--sensor-id",      type=int,   default=0)
    parser.add_argument("--capture-width",  type=int,   default=1280)
    parser.add_argument("--capture-height", type=int,   default=720)
    parser.add_argument("--width",          type=int,   default=1280)
    parser.add_argument("--height",         type=int,   default=720)
    parser.add_argument("--fps",            type=int,   default=60)
    parser.add_argument("--flip",           type=int,   default=0)
    parser.add_argument("--label",          type=str,   required=True,
                        choices=["free", "blocked"])
    parser.add_argument("--output-dir",     type=str,   default=None)
    parser.add_argument("--save-rate",      type=float, default=5.0)
    parser.add_argument("--duration",       type=float, default=60.0)
    parser.add_argument("--max-frames",     type=int,   default=0)
    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = os.path.expanduser(
            os.path.join("~", "data", args.label))
    args.output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    capture(args)


if __name__ == "__main__":
    main()