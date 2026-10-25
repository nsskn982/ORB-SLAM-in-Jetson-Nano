#!/usr/bin/env python3
"""
@file cnn_pipeline_monitor.py
@brief Verify CNN collision pipeline ROS topics are active.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float32


def wait_for_topic(topic, msg_type, timeout):
    """@brief Wait for a ROS topic message.
    @param topic Topic name.
    @param msg_type ROS message type.
    @param timeout Timeout seconds.
    @return True if message received, otherwise False.
    """
    try:
        rospy.wait_for_message(topic, msg_type, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def main():
    """@brief CLI entrypoint for pipeline monitoring."""
    parser = argparse.ArgumentParser(description="CNN pipeline monitor")
    parser.add_argument("--timeout", type=float, default=5.0, help="Timeout per topic")
    args = parser.parse_args()

    rospy.init_node("cnn_pipeline_monitor", anonymous=True)

    checks = [
        ("/pose", PoseStamped),
        ("/collision/probability", Float32),
        ("/collision/risky", Bool),
        ("/cmd_vel", Twist),
    ]

    failures = 0
    for topic, msg_type in checks:
        if wait_for_topic(topic, msg_type, args.timeout):
            print("OK: {0}".format(topic))
        else:
            print("WARN: no messages on {0}".format(topic))
            failures += 1

    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
