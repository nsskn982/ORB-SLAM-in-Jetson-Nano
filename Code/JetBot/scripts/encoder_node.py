#!/usr/bin/env python3
"""
@file encoder_node.py
@brief Wheel-encoder odometry node for the SparkFun JetBot v2.1.

@details
Reads left/right motor encoder ticks (/jetbot/encoder_ticks) and integrates
differential-drive kinematics to publish odometry (/odom) and a body twist
(/encoder/twist). The odometry supplements monocular ORB-SLAM2 during
low-texture / low-feature stretches where visual tracking is weak.

@author Srikanth Nanda
@date 2026-06-02
"""

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistStamped
from std_msgs.msg import Float32MultiArray


class EncoderNode(object):
    """@brief Publishes encoder-based odometry from JetBot v2.1 motors."""

    def __init__(self):
        """@brief Initialize parameters, state, publishers, subscriber, and timer."""
        # JetBot v2.1 parameters
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.066)  # meters
        self.wheelbase = rospy.get_param("~wheelbase", 0.090)  # distance between wheels
        self.ticks_per_revolution = rospy.get_param("~ticks_per_revolution", 64)
        self.encoder_topic = rospy.get_param("~encoder_topic", "/jetbot/encoder_ticks")
        self.publish_freq = rospy.get_param("~publish_freq", 20)  # Hz
        
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meters_per_tick = self.wheel_circumference / self.ticks_per_revolution
        
        # State
        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lock = threading.Lock()
        
        # Publishers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.twist_pub = rospy.Publisher("/encoder/twist", TwistStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(self.encoder_topic, Float32MultiArray, self._on_encoder_ticks)
        
        # Timer for odometry publishing
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_freq),
            self._publish_odometry
        )
        
        rospy.loginfo(
            f"Encoder node started. Wheelbase: {self.wheelbase}m, "
            f"Wheel diameter: {self.wheel_diameter}m"
        )

    def _on_encoder_ticks(self, msg):
        """@brief Receive encoder tick counts [left_ticks, right_ticks].
        @param msg ROS Float32MultiArray with two tick counts.
        """
        try:
            with self.lock:
                self.left_ticks = int(msg.data[0])
                self.right_ticks = int(msg.data[1])
        except (IndexError, ValueError) as e:
            rospy.logwarn(f"Error parsing encoder ticks: {e}")

    def _publish_odometry(self, event):
        """@brief Integrate encoder deltas and publish odometry + twist.
        @param event ROS Timer event (provides last_duration for dt).
        """
        with self.lock:
            delta_left = (self.left_ticks - self.prev_left_ticks) * self.meters_per_tick
            delta_right = (self.right_ticks - self.prev_right_ticks) * self.meters_per_tick
            
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
        
        if abs(delta_left) < 1e-6 and abs(delta_right) < 1e-6:
            return
        
        # Differential drive kinematics
        delta_center = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheelbase
        
        # Update pose (no lock needed; these are only written in this timer callback)
        self.theta += delta_theta
        self.x += delta_center * math.cos(self.theta - delta_theta / 2.0)
        self.y += delta_center * math.sin(self.theta - delta_theta / 2.0)
        
        x, y, theta = self.x, self.y, self.theta
        
        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        
        # Quaternion from yaw
        q = self._yaw_to_quaternion(theta)
        odom_msg.pose.pose.orientation = q
        
        # Covariance (encoder uncertainty grows over time)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # theta
        
        self.odom_pub.publish(odom_msg)
        
        # Also publish twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_link"
        dt = event.last_duration.to_sec() if event.last_duration and event.last_duration.to_sec() > 1e-6 else (1.0 / self.publish_freq)
        twist_msg.twist.linear.x = delta_center / dt
        twist_msg.twist.angular.z = delta_theta / dt
        self.twist_pub.publish(twist_msg)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        """@brief Convert a yaw angle to a quaternion.
        @param yaw Yaw angle in radians.
        @return geometry_msgs/Quaternion.
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("encoder_node", anonymous=False)
    EncoderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
