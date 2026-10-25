"""
@file imx219_camera_pipeline.py
@brief Complete camera bring-up, calibration, and usage pipeline for
       Leopard Imaging IMX219 MIPI FF NANO camera on Jetson Nano / Linux.

This module covers:
- Camera indicators and bring-up checks
- Frame capture
- Camera calibration using a checkerboard
- Saving and loading intrinsic parameters
- Real-time undistortion
- Example execution loop

Designed for OpenCV-based robotics and ROS 2 camera integration.
"""

import cv2
import numpy as np
import glob
import os


# ============================
# Camera Indicators (Concept)
# ============================
#
# - Power LED: ON when CSI camera is powered
# - Activity LED: Blinks during frame capture
# - CSI link health: Confirmed when VideoCapture opens successfully
# - Driver readiness: Failure raises runtime exceptions
#


##
# @brief Open the Leopard Imaging IMX219 camera.
#
# Verifies CSI link, driver availability, and stream readiness.
#
# @param device Camera device index (default: 0)
# @param width Desired image width
# @param height Desired image height
# @return cv2.VideoCapture object
#
def open_camera(device=0, width=1280, height=720):
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        raise RuntimeError(
            "IMX219 camera not detected. "
            "Check CSI connection, power LED, and drivers."
        )
    return cap


##
# @brief Capture a single frame from the camera.
#
# @param cap Active VideoCapture object
# @return Captured image frame
#
def capture_frame(cap):
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Frame capture failed (CSI link unstable).")
    return frame


##
# @brief Perform camera calibration using checkerboard images.
#
# @param image_dir Directory containing calibration images (*.jpg)
# @param checkerboard Number of inner corners (columns, rows)
# @param square_size Checker square size in meters
# @return (camera_matrix, distortion_coefficients)
#
def calibrate_camera(
    image_dir,
    checkerboard=(9, 6),
    square_size=0.025
):
    objp = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0:checkerboard[0],
        0:checkerboard[1]
    ].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []

    images = glob.glob(os.path.join(image_dir, "*.jpg"))
    if not images:
        raise RuntimeError("No calibration images found.")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard, None)
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    ret, mtx, dist, _, _ = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        None,
        None
    )

    np.savez("camera_calibration.npz", mtx=mtx, dist=dist)
    return mtx, dist


##
# @brief Load saved camera calibration parameters.
#
# @param filename Calibration file path
# @return (camera_matrix, distortion_coefficients)
#
def load_calibration(filename="camera_calibration.npz"):
    if not os.path.exists(filename):
        raise RuntimeError("Calibration file not found.")
    data = np.load(filename)
    return data["mtx"], data["dist"]


##
# @brief Undistort an image using camera intrinsics.
#
# @param frame Distorted input frame
# @param mtx Camera matrix
# @param dist Distortion coefficients
# @return Undistorted frame
#
def undistort_frame(frame, mtx, dist):
    return cv2.undistort(frame, mtx, dist)


##
# @brief Demonstration loop for IMX219 camera usage.
#
# - Captures frames
# - Applies undistortion
# - Displays live output
#
# Press 'q' to exit.
#
def main():
    cap = open_camera()
    mtx, dist = load_calibration()

    while True:
        frame = capture_frame(cap)
        corrected = undistort_frame(frame, mtx, dist)

        cv2.imshow("IMX219 Undistorted Output", corrected)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
