#!/usr/bin/env python3
"""
/**
 * @file MR.py
 * @brief Mobile Robot Control System (Camera + Motors + Odometry + ROS2)
 * @author Srikanth
 * @date 2025
 *
 * This script controls a Jetson-based mobile robot with:
 *  - Qwiic motor driver (abstracted here)
 *  - Wheel odometry
 *  - Camera capture
 *  - Calibration hooks
 *  - Testing utilities
 *  - Optional ROS2 node interface (cmd_vel / odom)
 */
"""

import time
import math
import cv2
import numpy as np

# ROS2 imports are optional; code can still run in standalone mode
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time as RosTime
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# --------------------------------------------------------------------
# CAMERA SETUP
# --------------------------------------------------------------------
def init_camera(index=0, width=1280, height=720):
    """Initialize a simple video capture for Jetson."""
    cam = cv2.VideoCapture(index)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cam.isOpened():
        print("[Camera] Warning: could not open camera index", index)
    return cam


# --------------------------------------------------------------------
# MOTOR DRIVER MOCKUP
# --------------------------------------------------------------------
class MotorDriver:
    """
    Simple motor driver placeholder.
    Replace with a real Qwiic Motor Driver API.
    """

    def __init__(self):
        self.left_speed = 0.0
        self.right_speed = 0.0

    def set_speeds(self, left, right):
        """
        Set motor speeds (for now just prints; in real code, send to driver).
        """
        self.left_speed = left
        self.right_speed = right
        print(f"[Motor] Left={left:.2f}, Right={right:.2f}")

    def stop(self):
        """Stop both motors."""
        self.set_speeds(0.0, 0.0)


# --------------------------------------------------------------------
# ODOMETRY
# --------------------------------------------------------------------
class Odometry:
    """
    Differential-drive odometry model.
    """

    def __init__(self, wheel_radius=0.033, wheel_base=0.17):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.r = wheel_radius
        self.L = wheel_base

    def update(self, wl, wr, dt):
        """
        Update pose given left/right angular velocities (rad/s).
        """
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        return self.x, self.y, self.theta


# --------------------------------------------------------------------
# CALIBRATION ROUTINES (PLACEHOLDERS)
# --------------------------------------------------------------------
def camera_calibration(image_paths):
    """
    Camera calibration placeholder.
    """
    print("[Calibration] Running camera calibration on image set...")
    # Real implementation would use cv2.findChessboardCorners, etc.
    K = np.eye(3)
    D = np.zeros(5)
    print("[Calibration] Using identity K and zero distortion as placeholder.")
    return {"K": K, "D": D}


def motor_calibration(motor):
    """
    Simple calibration hook: run motors forward briefly.
    """
    print("[Calibration] Motor calibration: forward test...")
    motor.set_speeds(50.0, 50.0)
    time.sleep(1.0)
    motor.stop()
    print("[Calibration] Motor calibration: done.")


# --------------------------------------------------------------------
# TEST FUNCTIONS
# --------------------------------------------------------------------
def test_motors(motor):
    """
    Sweep motor speeds to verify wiring and direction.
    """
    print("[Test] Sweeping motor speeds...")
    for s in [30, 60, 100, 0]:
        motor.set_speeds(s, s)
        time.sleep(0.5)
    motor.stop()
    print("[Test] Motor test complete.")


def test_camera(camera):
    """
    Capture a single test frame and save to disk.
    """
    print("[Test] Capturing one test frame from camera...")
    ret, frame = camera.read()
    if ret:
        cv2.imwrite("test_frame.jpg", frame)
        print("[Test] Saved test_frame.jpg")
    else:
        print("[Test] Failed to capture frame.")


# --------------------------------------------------------------------
# STANDALONE MAIN LOOP (NON-ROS2)
# --------------------------------------------------------------------
def run_standalone():
    """
    Standalone mode without ROS2.
    """
    print("[Standalone] Starting non-ROS2 robot test...")

    cam = init_camera()
    motor = MotorDriver()
    odo = Odometry()

    motor_calibration(motor)
    test_motors(motor)
    test_camera(cam)

    print("[Standalone] Driving slowly forward with odometry update...")
    motor.set_speeds(50.0, 50.0)

    t_prev = time.time()
    for _ in range(20):
        t_now = time.time()
        dt = t_now - t_prev
        t_prev = t_now

        # Assume constant wheel angular velocity for demonstration
        wl = 4.0
        wr = 4.0
        x, y, th = odo.update(wl, wr, dt)
        print(f"[Odom] x={x:.3f} m, y={y:.3f} m, theta={th:.3f} rad")
        time.sleep(0.1)

    motor.stop()
    print("[Standalone] Done.")


# --------------------------------------------------------------------
# ROS2 NODE
# --------------------------------------------------------------------
class MobileRobotNode(Node):
    """
    ROS2 node controlling the mobile robot.

    Subscribes:
      - /cmd_vel (geometry_msgs/Twist)

    Publishes:
      - /odom (nav_msgs/Odometry)
    """

    def __init__(self):
        super().__init__("mobile_robot_node")
        self.motor = MotorDriver()
        self.odo = Odometry()

        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.current_cmd = Twist()

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/odom", 10
        )

        self.timer = self.create_timer(0.05, self.update_loop)
        self.get_logger().info("MobileRobotNode started.")

    def cmd_vel_callback(self, msg):
        """
        Handle incoming velocity commands.
        """
        self.current_cmd = msg

    def update_loop(self):
        """
        Periodic update: odometry + motor command.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        v = self.current_cmd.linear.x
        w = self.current_cmd.angular.z

        # Convert v, w to wheel angular velocities
        wl = (2.0 * v - w * self.odo.L) / (2.0 * self.odo.r)
        wr = (2.0 * v + w * self.odo.L) / (2.0 * self.odo.r)

        # Update odometry
        x, y, theta = self.odo.update(wl, wr, dt)

        # Simple mapping from desired v to motor "speed" units
        left_cmd = max(min(v * 100.0, 100.0), -100.0)
        right_cmd = max(min(v * 100.0, 100.0), -100.0)
        self.motor.set_speeds(left_cmd, right_cmd)

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        stamp = self.get_clock().now().to_msg()
        odom_msg.header.stamp = stamp

        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0

        # Orientation as yaw-only quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        self.odom_pub.publish(odom_msg)


def run_ros2():
    """
    Run the robot as a ROS2 node.
    """
    if not ROS2_AVAILABLE:
        print("[ROS2] rclpy or messages not available. Install ROS2 first.")
        return

    rclpy.init()
    node = MobileRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.motor.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("[ROS2] Node shut down.")


# --------------------------------------------------------------------
# ENTRY POINT
# --------------------------------------------------------------------
if __name__ == "__main__":
    # Choose whether to run standalone or ROS2 mode.
    # For a real project you could parse command line arguments.
    USE_ROS2 = False

    if USE_ROS2:
        run_ros2()
    else:
        run_standalone()