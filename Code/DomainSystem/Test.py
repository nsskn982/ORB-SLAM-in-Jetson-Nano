#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file   Test.py
@brief  Domain system tests for JetBot hardware and software.
@author Srikanth Nanda
@date   2026-05-15

This script implements hardware and software tests for the SparkFun
JetBot platform based on the NVIDIA Jetson Nano. It verifies camera
capture, motor driver connectivity, ROS topic availability, topic rates,
and TF transforms before running experiments.
"""

import time

import cv2
import smbus2

import rospy
import rosgraph
import tf2_ros

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


"""
@brief Default I2C bus on Jetson Nano.
"""
I2C_BUS_ID = 1

"""
@brief Default I2C address of the SparkFun Qwiic Motor Driver.
"""
MOTOR_DRIVER_ADDR = 0x5D

"""
@brief Required ROS topics for the domain system.
"""
REQUIRED_TOPICS = [
    "/camera/image_raw",
    "/pose",
    "/collision/probability",
    "/cmd_vel",
]

"""
@brief Base and camera frames used for TF validation.
"""
BASE_FRAME = "base_link"
CAMERA_FRAME = "camera_link"


def test_camera_capture(duration_s=5.0):
    """
    @brief Test IMX219 camera capture via a Jetson-compatible GStreamer pipeline.

    This function opens the CSI camera using the nvarguscamerasrc pipeline,
    captures frames for a short duration, and estimates the delivered frame rate.

    @param duration_s Duration in seconds for which frames are sampled.
    @return True if the camera opens and frames are captured, otherwise False.
    """
    pipeline = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1640, height=1232, format=NV12, framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=640, height=480, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! appsink"
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("[HW] Camera test FAILED: unable to open IMX219 pipeline")
        return False

    frame_count = 0
    frame = None
    start_time = time.time()

    while time.time() - start_time < duration_s:
        ret, frame = cap.read()
        if not ret:
            print("[HW] Camera test WARNING: frame capture failed")
            break
        frame_count += 1

    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0.0
    cap.release()

    if frame_count == 0:
        print("[HW] Camera test FAILED: no frames captured")
        return False

    print(f"[HW] Camera test OK: {frame_count} frames in {elapsed:.2f}s (~{fps:.2f} FPS)")

    if frame is not None:
        h, w = frame.shape[:2]
        print(f"[HW] Camera resolution: {w}x{h}")
        if w != 640 or h != 480:
            print("[HW] Camera test WARNING: resolution differs from expected 640x480")

    return True


def test_motor_driver_i2c():
    """
    @brief Test SparkFun Qwiic Motor Driver communication over I2C.

    This function checks whether the motor driver responds at its default
    I2C address on the expected bus.

    @return True if the motor driver responds, otherwise False.
    """
    try:
        bus = smbus2.SMBus(I2C_BUS_ID)
    except FileNotFoundError:
        print(f"[HW] Motor driver test FAILED: I2C bus {I2C_BUS_ID} not found")
        return False

    try:
        _ = bus.read_byte_data(MOTOR_DRIVER_ADDR, 0x00)
        print(f"[HW] Motor driver test OK: device found at 0x{MOTOR_DRIVER_ADDR:02X}")
        bus.close()
        return True
    except OSError:
        print(f"[HW] Motor driver test FAILED: no response at 0x{MOTOR_DRIVER_ADDR:02X}")
        bus.close()
        return False


def test_ros_master():
    """
    @brief Check whether the ROS master is reachable.

    @return True if the ROS master is online, otherwise False.
    """
    if not rosgraph.is_master_online():
        print("[SW] ROS master test FAILED: ROS master is not reachable")
        return False

    print("[SW] ROS master test OK: ROS master is reachable")
    return True


def test_ros_topics():
    """
    @brief Verify that required ROS topics are present.

    This function queries the ROS master and checks whether the required
    domain-system topics are currently published.

    @return True if all required topics are available, otherwise False.
    """
    master = rosgraph.Master("/domain_system_tester")

    try:
        state = master.getSystemState()
    except rosgraph.masterapi.Error:
        print("[SW] ROS topics test FAILED: unable to query ROS master")
        return False

    publishers, _, _ = state
    published_topics = {topic[0] for topic in publishers}

    missing = [topic for topic in REQUIRED_TOPICS if topic not in published_topics]

    if missing:
        print("[SW] ROS topics test FAILED: missing topics -> " + ", ".join(missing))
        return False

    print("[SW] ROS topics test OK: all required topics are present")
    return True


def measure_topic_rate(topic_name, msg_type, duration_s=5.0):
    """
    @brief Measure the approximate publication rate of a ROS topic.

    @param topic_name Name of the ROS topic.
    @param msg_type Message type associated with the topic.
    @param duration_s Measurement duration in seconds.
    @return Approximate topic frequency in Hz.
    """
    counter = {"n": 0}

    def callback(_msg):
        counter["n"] += 1

    subscriber = rospy.Subscriber(topic_name, msg_type, callback, queue_size=10)
    start_time = time.time()
    rospy.sleep(duration_s)
    subscriber.unregister()

    elapsed = time.time() - start_time
    if elapsed <= 0:
        return 0.0

    return counter["n"] / elapsed


def test_topic_rates():
    """
    @brief Test the publication rates of key ROS topics.

    This function measures the rates of the image and command topics and
    compares them with a minimal expected threshold.

    @return True if the measured rates are acceptable, otherwise False.
    """
    ok = True

    if not rospy.core.is_initialized():
        rospy.init_node("domain_system_topic_tester", anonymous=True)

    image_rate = measure_topic_rate("/camera/image_raw", Image, duration_s=5.0)
    print(f"[SW] /camera/image_raw rate ~ {image_rate:.2f} Hz")
    if image_rate < 5.0:
        print("[SW] Topic rate WARNING: /camera/image_raw below 5 Hz")
        ok = False

    cmd_rate = measure_topic_rate("/cmd_vel", Twist, duration_s=5.0)
    print(f"[SW] /cmd_vel rate ~ {cmd_rate:.2f} Hz")
    if cmd_rate < 1.0:
        print("[SW] Topic rate WARNING: /cmd_vel below 1 Hz")
        ok = False

    return ok


def test_tf_transform():
    """
    @brief Test whether the transform from base frame to camera frame exists.

    This function queries the TF buffer for the transform between the base
    frame and the camera frame within a timeout window.

    @return True if the transform is available, otherwise False.
    """
    if not rospy.core.is_initialized():
        rospy.init_node("domain_system_tf_tester", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    _listener = tf2_ros.TransformListener(tf_buffer)

    timeout_s = 5.0
    start_time = time.time()

    while not rospy.is_shutdown() and time.time() - start_time < timeout_s:
        try:
            tf_buffer.lookup_transform(
                BASE_FRAME,
                CAMERA_FRAME,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            print(f"[SW] TF test OK: transform {BASE_FRAME} -> {CAMERA_FRAME} available")
            return True
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)

    print(f"[SW] TF test FAILED: transform {BASE_FRAME} -> {CAMERA_FRAME} not available")
    return False


def run_all_tests():
    """
    @brief Run all hardware and software domain-system tests.

    This function orchestrates the complete test sequence and prints a
    summary of pass/fail status for hardware and software.

    @return None
    """
    print("=== Domain System Tests: Hardware ===")
    hardware_ok = True
    hardware_ok &= test_camera_capture(duration_s=5.0)
    hardware_ok &= test_motor_driver_i2c()

    print("\n=== Domain System Tests: Software ===")
    software_ok = True
    software_ok &= test_ros_master()

    if software_ok:
        if not rospy.core.is_initialized():
            rospy.init_node("domain_system_tester", anonymous=True)
        software_ok &= test_ros_topics()
        software_ok &= test_topic_rates()
        software_ok &= test_tf_transform()

    print("\n=== Test Summary ===")
    print(f"Hardware tests: {'PASS' if hardware_ok else 'FAIL'}")
    print(f"Software tests: {'PASS' if software_ok else 'FAIL'}")

    if hardware_ok and software_ok:
        print("Overall result: PASS")
    else:
        print("Overall result: FAIL")


if __name__ == "__main__":
    run_all_tests()