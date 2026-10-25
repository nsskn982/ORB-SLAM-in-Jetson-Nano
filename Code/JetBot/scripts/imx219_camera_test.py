#!/usr/bin/env python3
"""
@file imx219_camera_test.py
@brief Live test utility for Leopard Imaging IMX219 camera streams on Jetson.
@author Srikanth Nanda
@date 2026-05-08
"""

import argparse
import os
import time

import cv2


def build_gstreamer_pipeline(
    sensor_id,
    capture_width,
    capture_height,
    output_width,
    output_height,
    fps,
    flip,
):
    """@brief Build a GStreamer pipeline string for CSI camera capture.
    @param sensor_id CSI sensor index.
    @param capture_width Sensor capture width.
    @param capture_height Sensor capture height.
    @param output_width Output width.
    @param output_height Output height.
    @param fps Frames per second.
    @param flip Flip method for nvvidconv.
    @return Pipeline string.
    """
    return (
        "nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        "format=(string)NV12, framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method={flip} ! "
        "video/x-raw, width=(int){output_width}, height=(int){output_height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1"
    ).format(
        sensor_id=sensor_id,
        capture_width=capture_width,
        capture_height=capture_height,
        output_width=output_width,
        output_height=output_height,
        fps=fps,
        flip=flip,
    )


def open_capture(args):
    """@brief Open a VideoCapture for CSI, V4L2, or file sources.
    @param args Parsed CLI arguments.
    @return Tuple of (cv2.VideoCapture, source description).
    """
    if args.source == "csi":
        pipeline = build_gstreamer_pipeline(
            sensor_id=args.sensor_id,
            capture_width=args.capture_width,
            capture_height=args.capture_height,
            output_width=args.width,
            output_height=args.height,
            fps=args.fps,
            flip=args.flip,
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        return cap, pipeline
    if args.source == "v4l2":
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        return cap, "v4l2:///dev/video{0}".format(args.device)
    cap = cv2.VideoCapture(args.file)
    return cap, args.file


def save_frame(frame, save_dir, prefix):
    """@brief Save a snapshot frame to disk.
    @param frame BGR image to save.
    @param save_dir Output directory.
    @param prefix Filename prefix.
    @return Full path to saved image.
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = "{0}_{1}.png".format(prefix, timestamp)
    path = os.path.join(save_dir, filename)
    cv2.imwrite(path, frame)
    return path


def main():
    """@brief CLI entrypoint for IMX219 live test."""
    parser = argparse.ArgumentParser(description="IMX219 camera live test")
    parser.add_argument("--source", choices=["csi", "v4l2", "file"], default="csi")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--capture-width", type=int, default=3280)
    parser.add_argument("--capture-height", type=int, default=2464)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", type=int, default=0)
    parser.add_argument("--device", type=int, default=0, help="/dev/video index for V4L2")
    parser.add_argument("--file", type=str, default="", help="Video file path for file source")
    parser.add_argument("--no-display", action="store_true", help="Disable preview window")
    parser.add_argument("--save-dir", type=str, default="", help="Directory to save snapshots")
    parser.add_argument("--snapshot-prefix", type=str, default="imx219")
    parser.add_argument("--window-name", type=str, default="IMX219 Test")
    args = parser.parse_args()

    if args.source == "file" and not args.file:
        raise SystemExit("--file is required when source=file")

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    cap, source_desc = open_capture(args)
    if not cap.isOpened():
        raise SystemExit("Failed to open camera source: {0}".format(source_desc))

    print("Opened source:", source_desc)
    last_fps_time = time.time()
    frame_count = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed. Exiting.")
            break

        frame_count += 1
        now = time.time()
        elapsed = now - last_fps_time
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            last_fps_time = now

        if not args.no_display:
            overlay = "{0}x{1}  {2:.1f} FPS".format(frame.shape[1], frame.shape[0], fps)
            cv2.putText(
                frame,
                overlay,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow(args.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
        else:
            key = 255

        if key in (ord("q"), 27):
            break
        if key == ord("s") and args.save_dir:
            path = save_frame(frame, args.save_dir, args.snapshot_prefix)
            print("Saved snapshot:", path)

    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
