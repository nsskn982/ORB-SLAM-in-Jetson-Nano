#!/usr/bin/env python3
"""
@file telemetry_logger_node.py
@brief Telemetry logger for the SparkFun JetBot v2.1-based mobile robot.

@details
Subscribes to pose, odometry, command velocity, collision probability/flag, and
battery topics, and writes a time-stamped row to a CSV file at a fixed rate for
post-mission analysis. The logger is passive and does not interfere with the
control pipeline.

@author Srikanth Nanda
@date 2026-06-02
"""

import math
import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry


class TelemetryLoggerNode(object):
    """@brief Logs telemetry data to a CSV file."""

    def __init__(self):
        """@brief Initialize parameters, CSV file, subscribers, and timer."""
        self.log_dir = rospy.get_param("~log_dir", os.path.expanduser("~/jetbot_logs"))
        self.enable_logging = rospy.get_param("~enable_logging", True)
        
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create log file with timestamp
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"telemetry_{timestamp}.csv")
        self.csv_file = None
        self.csv_writer = None
        
        # Initialize CSV
        self._init_csv()
        
        # Subscribers
        rospy.Subscriber("/pose", PoseStamped, self._on_pose)
        rospy.Subscriber("/odom", Odometry, self._on_odom)
        rospy.Subscriber("/cmd_vel", TwistStamped, self._on_cmd_vel)
        rospy.Subscriber("/collision/probability", Float32, self._on_collision_prob)
        rospy.Subscriber("/collision/risky", Bool, self._on_collision_risky)
        rospy.Subscriber("/battery/voltage", Float32, self._on_battery_voltage)
        rospy.Subscriber("/battery/soc", Float32, self._on_battery_soc)
        
        # Current state
        self.state = {
            "timestamp": 0.0,
            "pose_x": 0.0,
            "pose_y": 0.0,
            "pose_z": 0.0,
            "odom_x": 0.0,
            "odom_y": 0.0,
            "odom_theta": 0.0,
            "cmd_vel_x": 0.0,
            "cmd_vel_z": 0.0,
            "collision_prob": 0.0,
            "collision_risky": False,
            "battery_voltage": 0.0,
            "battery_soc": 0.0,
        }
        
        # Timer for periodic logging
        rospy.Timer(rospy.Duration(1.0), self._log_state)
        
        rospy.loginfo(f"Telemetry logger started. Log file: {self.log_file}")

    def _init_csv(self):
        """@brief Open the CSV file and write the header row."""
        try:
            self.csv_file = open(self.log_file, "w", newline="")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    "timestamp",
                    "pose_x", "pose_y", "pose_z",
                    "odom_x", "odom_y", "odom_theta",
                    "cmd_vel_x", "cmd_vel_z",
                    "collision_prob", "collision_risky",
                    "battery_voltage", "battery_soc",
                ],
            )
            self.csv_writer.writeheader()
            self.csv_file.flush()
        except Exception as e:
            rospy.logerr(f"Error initializing CSV: {e}")
            self.csv_writer = None

    def _on_pose(self, msg):
        """@brief Cache the SLAM pose.
        @param msg ROS PoseStamped message.
        """
        self.state["timestamp"] = msg.header.stamp.to_sec()
        self.state["pose_x"] = msg.pose.position.x
        self.state["pose_y"] = msg.pose.position.y
        self.state["pose_z"] = msg.pose.position.z

    def _on_odom(self, msg):
        """@brief Cache encoder odometry and extract yaw.
        @param msg ROS Odometry message.
        """
        self.state["odom_x"] = msg.pose.pose.position.x
        self.state["odom_y"] = msg.pose.pose.position.y
        # Extract yaw from quaternion (full atan2 formula)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.state["odom_theta"] = math.atan2(siny_cosp, cosy_cosp)

    def _on_cmd_vel(self, msg):
        """@brief Cache the latest velocity command.
        @param msg ROS TwistStamped message.
        """
        self.state["cmd_vel_x"] = msg.twist.linear.x
        self.state["cmd_vel_z"] = msg.twist.angular.z

    def _on_collision_prob(self, msg):
        """@brief Cache the collision probability.
        @param msg ROS Float32 message.
        """
        self.state["collision_prob"] = msg.data

    def _on_collision_risky(self, msg):
        """@brief Cache the collision risky flag.
        @param msg ROS Bool message.
        """
        self.state["collision_risky"] = msg.data

    def _on_battery_voltage(self, msg):
        """@brief Cache the battery voltage.
        @param msg ROS Float32 message.
        """
        self.state["battery_voltage"] = msg.data

    def _on_battery_soc(self, msg):
        """@brief Cache the battery state of charge.
        @param msg ROS Float32 message.
        """
        self.state["battery_soc"] = msg.data

    def _log_state(self, event):
        """@brief Write the current cached state to the CSV file.
        @param event ROS Timer event (unused).
        """
        if not self.enable_logging or self.csv_writer is None:
            return
        
        try:
            self.csv_writer.writerow(self.state)
            self.csv_file.flush()
        except Exception as e:
            rospy.logerr(f"Error writing CSV: {e}")

    def run(self):
        """@brief Keep the node alive and close the CSV file on shutdown."""
        try:
            rospy.spin()
        finally:
            if self.csv_file:
                self.csv_file.close()
                rospy.loginfo(f"Telemetry log closed: {self.log_file}")


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("telemetry_logger_node", anonymous=False)
    node = TelemetryLoggerNode()
    node.run()


if __name__ == "__main__":
    main()
