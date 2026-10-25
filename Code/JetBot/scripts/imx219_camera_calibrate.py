#!/usr/bin/env python3
"""
@file imx219_camera_calibrate.py
@brief Chessboard-based intrinsic calibration utility for IMX219 cameras.
@author Srikanth Nanda
@date 2026-05-08
"""

import argparse
import os
import time

import cv2
import numpy as np


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


def build_object_points(board_cols, board_rows, square_size):
    """@brief Build 3D object points for the chessboard pattern.
    @param board_cols Number of inner corners along width.
    @param board_rows Number of inner corners along height.
    @param square_size Size of a square in meters.
    @return Object points array.
    """
    objp = np.zeros((board_rows * board_cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    """@brief Compute mean reprojection error for calibration.
    @param objpoints List of object points.
    @param imgpoints List of image points.
    @param rvecs Rotation vectors.
    @param tvecs Translation vectors.
    @param mtx Camera matrix.
    @param dist Distortion coefficients.
    @return Mean reprojection error.
    """
    total_error = 0.0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error
    return total_error / max(1, len(objpoints))


def write_calibration(output_path, image_size, mtx, dist, rms, reproj_error, args):
    """@brief Write calibration results to a YAML file.
    @param output_path Output file path.
    @param image_size Image size (width, height).
    @param mtx Camera matrix.
    @param dist Distortion coefficients.
    @param rms RMS reprojection error from OpenCV.
    @param reproj_error Mean reprojection error.
    @param args Parsed CLI arguments.
    """
    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("rms", float(rms))
    fs.write("reprojection_error", float(reproj_error))
    fs.write("board_cols", int(args.board_cols))
    fs.write("board_rows", int(args.board_rows))
    fs.write("square_size", float(args.square_size))
    fs.write("calibration_date", time.strftime("%Y-%m-%d"))
    fs.release()


def main():
    """@brief CLI entrypoint for IMX219 calibration."""
    parser = argparse.ArgumentParser(description="IMX219 camera calibration")
    parser.add_argument("--source", choices=["csi", "v4l2", "file"], default="csi")
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--capture-width", type=int, default=3280)
    parser.add_argument("--capture-height", type=int, default=2464)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--flip", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--file", type=str, default="")
    parser.add_argument("--board-cols", type=int, default=9)
    parser.add_argument("--board-rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.024)
    parser.add_argument("--num-frames", type=int, default=20)
    parser.add_argument("--output", type=str, default="config/imx219_calibration.yaml")
    parser.add_argument("--window-name", type=str, default="IMX219 Calibration")
    args = parser.parse_args()

    if args.source == "file" and not args.file:
        raise SystemExit("--file is required when source=file")

    cap, source_desc = open_capture(args)
    if not cap.isOpened():
        raise SystemExit("Failed to open camera source: {0}".format(source_desc))

    print("Opened source:", source_desc)
    objp = build_object_points(args.board_cols, args.board_rows, args.square_size)
    objpoints = []
    imgpoints = []
    image_size = None

    print("Press 'c' to capture a frame, 'r' to reset, 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed. Exiting.")
            break

        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            (args.board_cols, args.board_rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        display = frame.copy()
        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            cv2.drawChessboardCorners(display, (args.board_cols, args.board_rows), corners, found)

        status = "Captured: {0}/{1}".format(len(objpoints), args.num_frames)
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(args.window_name, display)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            objpoints = []
            imgpoints = []
            print("Reset captured frames.")
        if key == ord("c") and found:
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            print("Captured frame {0}/{1}.".format(len(objpoints), args.num_frames))

        if len(objpoints) >= args.num_frames:
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 5:
        raise SystemExit("Not enough frames captured for calibration (need at least 5).")

    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )
    reproj_error = compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist)

    write_calibration(args.output, image_size, mtx, dist, rms, reproj_error, args)

    print("Calibration complete. RMS: {0:.4f} Reprojection error: {1:.4f}".format(rms, reproj_error))
    print("Saved calibration to:", args.output)


if __name__ == "__main__":
    main()
