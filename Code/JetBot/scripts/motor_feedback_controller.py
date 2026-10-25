#!/usr/bin/env python3
"""
@file motor_feedback_controller.py
@brief PID velocity feedback controller for the SparkFun JetBot v2.1.

@details
Subscribes to the decision node command (/cmd_vel) and encoder ticks
(/jetbot/encoder_ticks), runs per-wheel PID feedback to track the target
wheel velocities, and publishes corrected motor commands
(/jetbot/motor_commands) plus a diagnostic JointState (/motor/joint_state).
Reduces wheel slip and skid-steer drift on the differential-drive chassis.

@author Srikanth Nanda
@date 2026-06-02
"""

import threading

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from jetbot_utils import clamp


class MotorFeedbackController(object):
    """@brief PID feedback controller for motor velocity correction."""

    def __init__(self):
        """@brief Initialize parameters, PID state, publishers, subscribers, and timer."""
        # Control parameters
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.066)  # meters
        self.wheelbase = rospy.get_param("~wheelbase", 0.090)  # meters
        self.ticks_per_revolution = rospy.get_param("~ticks_per_revolution", 64)
        
        # PID gains
        self.kp = rospy.get_param("~kp", 0.5)
        self.ki = rospy.get_param("~ki", 0.1)
        self.kd = rospy.get_param("~kd", 0.05)
        
        # Control parameters
        self.max_correction = rospy.get_param("~max_correction", 0.1)  # max velocity adjustment
        self.publish_freq = rospy.get_param("~publish_freq", 20)  # Hz
        self.meters_per_tick = (3.14159 * self.wheel_diameter) / self.ticks_per_revolution
        
        # State
        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = rospy.Time.now()
        
        # PID state
        self.left_integral_error = 0.0
        self.right_integral_error = 0.0
        self.left_prev_error = 0.0
        self.right_prev_error = 0.0
        self.lock = threading.Lock()
        
        # Publishers
        self.motor_cmd_pub = rospy.Publisher(
            "/jetbot/motor_commands", Float32MultiArray, queue_size=10
        )
        self.joint_state_pub = rospy.Publisher(
            "/motor/joint_state", JointState, queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber("/cmd_vel", TwistStamped, self._on_cmd_vel)
        rospy.Subscriber("/jetbot/encoder_ticks", Float32MultiArray, self._on_encoder_ticks)
        
        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.publish_freq), self._feedback_control)
        
        rospy.loginfo(
            f"Motor feedback controller started. KP={self.kp}, KI={self.ki}, KD={self.kd}"
        )

    def _on_cmd_vel(self, msg):
        """@brief Receive the target velocity command.
        @param msg ROS TwistStamped command.
        """
        with self.lock:
            self.cmd_linear_x = msg.twist.linear.x
            self.cmd_angular_z = msg.twist.angular.z

    def _on_encoder_ticks(self, msg):
        """@brief Receive encoder ticks [left, right].
        @param msg ROS Float32MultiArray with two tick counts.
        """
        try:
            with self.lock:
                self.left_ticks = int(msg.data[0])
                self.right_ticks = int(msg.data[1])
        except IndexError:
            pass

    def _feedback_control(self, event):
        """@brief Run per-wheel PID feedback and publish corrected commands.
        @param event ROS Timer event (unused).
        """
        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        
        if dt < 0.01:  # Minimum time step
            return
        
        with self.lock:
            cmd_linear = self.cmd_linear_x
            cmd_angular = self.cmd_angular_z
            delta_left = (self.left_ticks - self.prev_left_ticks) * self.meters_per_tick
            delta_right = (self.right_ticks - self.prev_right_ticks) * self.meters_per_tick
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
        
        self.prev_time = now
        
        # Calculate target and actual wheel speeds
        target_left = cmd_linear - cmd_angular * self.wheelbase / 2.0
        target_right = cmd_linear + cmd_angular * self.wheelbase / 2.0
        
        actual_left = delta_left / dt
        actual_right = delta_right / dt
        
        # PID control
        left_error = target_left - actual_left
        right_error = target_right - actual_right
        
        self.left_integral_error = clamp(
            self.left_integral_error + left_error * dt, -1.0, 1.0
        )
        self.right_integral_error = clamp(
            self.right_integral_error + right_error * dt, -1.0, 1.0
        )
        
        left_derivative = (left_error - self.left_prev_error) / dt if dt > 0 else 0
        right_derivative = (right_error - self.right_prev_error) / dt if dt > 0 else 0
        
        self.left_prev_error = left_error
        self.right_prev_error = right_error
        
        # Calculate corrections
        left_correction = (
            self.kp * left_error +
            self.ki * self.left_integral_error +
            self.kd * left_derivative
        )
        right_correction = (
            self.kp * right_error +
            self.ki * self.right_integral_error +
            self.kd * right_derivative
        )
        
        left_correction = clamp(left_correction, -self.max_correction, self.max_correction)
        right_correction = clamp(right_correction, -self.max_correction, self.max_correction)
        
        # Corrected commands
        left_cmd = clamp(target_left + left_correction, -1.0, 1.0)
        right_cmd = clamp(target_right + right_correction, -1.0, 1.0)
        
        # Publish motor commands
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(left_cmd), float(right_cmd)]
        self.motor_cmd_pub.publish(motor_msg)
        
        # Publish joint state for diagnostics
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = ["left_motor", "right_motor"]
        joint_msg.velocity = [actual_left, actual_right]
        joint_msg.effort = [left_cmd, right_cmd]
        self.joint_state_pub.publish(joint_msg)


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("motor_feedback_controller", anonymous=False)
    MotorFeedbackController()
    rospy.spin()


if __name__ == "__main__":
    main()
