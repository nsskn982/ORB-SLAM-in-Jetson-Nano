#!/usr/bin/env python3
"""
@file cnn_data_collect.py
@brief Collect images, pose, and collision probability for CNN datasets.
@author Srikanth Nanda
@date 2026-05-16
"""

import argparse
import csv
import os
import threading
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


SUPPORTED_ENCODINGS = ("bgr8", "rgb8", "mono8")


def ros_image_to_bgr(msg):
    """@brief Convert ROS Image to BGR numpy array.
    @param msg ROS Image message.
    @return BGR image.
    """
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError("Invalid image size")

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    height = int(msg.height)
    width = int(msg.width)

    if msg.encoding in ("bgr8", "rgb8"):
        needed = height * width * 3
        if raw.size < needed:
            raise ValueError("Insufficient RGB/BGR data")
        image = raw[:needed].reshape((height, width, 3))
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    if msg.encoding == "mono8":
        needed = height * width
        if raw.size < needed:
            raise ValueError("Insufficient mono data")
        mono = raw[:needed].reshape((height, width))
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

    raise ValueError("Unsupported encoding: {0}".format(msg.encoding))


class DataCollector(object):
    def __init__(self, output_dir, image_topic, pose_topic, collision_topic, save_rate, duration, max_frames):
        """@brief Initialize data collection.
        @param output_dir Output directory for images and CSV.
        @param image_topic ROS image topic.
        @param pose_topic ROS pose topic.
        @param collision_topic ROS collision probability topic.
        @param save_rate Target save rate (Hz).
        @param duration Collection duration seconds (0 for infinite).
        @param max_frames Maximum frames to save (0 for infinite).
        """
        self.output_dir = output_dir
        self.images_dir = os.path.join(output_dir, "images")
        self.csv_path = os.path.join(output_dir, "data.csv")
        os.makedirs(self.images_dir, exist_ok=True)

        self.save_period = 1.0 / max(1e-6, save_rate)
        self.max_frames = max_frames
        self.end_time = None
        if duration > 0.0:
            self.end_time = rospy.Time.now() + rospy.Duration(duration)

        self.last_save_time = 0.0
        self.saved_frames = 0
        self.last_pose = None
        self.last_prob = None
        self.lock = threading.Lock()

        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp",
            "image",
            "pose_x",
            "pose_y",
            "pose_z",
            "pose_qx",
            "pose_qy",
            "pose_qz",
            "pose_qw",
            "collision_prob",
        ])

        self.image_sub = rospy.Subscriber(image_topic, Image, self._image_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self._pose_callback, queue_size=1)
        self.collision_sub = rospy.Subscriber(collision_topic, Float32, self._collision_callback, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

    def _pose_callback(self, msg):
        """@brief Store latest pose message.
        @param msg ROS PoseStamped message.
        """
        self.last_pose = msg

    def _collision_callback(self, msg):
        """@brief Store latest collision probability.
        @param msg ROS Float32 message.
        """
        self.last_prob = float(msg.data)

    def _image_callback(self, msg):
        """@brief Save images and metadata at the target rate.
        @param msg ROS Image message.
        """
        if msg.encoding not in SUPPORTED_ENCODINGS:
            return

        now = time.time()
        if now - self.last_save_time < self.save_period:
            return

        if self.end_time and rospy.Time.now() >= self.end_time:
            rospy.signal_shutdown("Collection duration reached")
            return

        if self.max_frames and self.saved_frames >= self.max_frames:
            rospy.signal_shutdown("Max frames reached")
            return

        try:
            image = ros_image_to_bgr(msg)
        except Exception:
            return

        timestamp = rospy.Time.now().to_nsec()
        filename = "frame_{0}.png".format(timestamp)
        image_path = os.path.join(self.images_dir, filename)

        if not cv2.imwrite(image_path, image):
            return

        with self.lock:
            pose = self.last_pose.pose if self.last_pose else None
            pose_values = ["", "", "", "", "", "", ""]
            if pose:
                pose_values = [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            prob = self.last_prob if self.last_prob is not None else ""
            self.csv_writer.writerow([
                rospy.Time.now().to_sec(),
                os.path.join("images", filename),
                pose_values[0],
                pose_values[1],
                pose_values[2],
                pose_values[3],
                pose_values[4],
                pose_values[5],
                pose_values[6],
                prob,
            ])
            self.csv_file.flush()

        self.last_save_time = now
        self.saved_frames += 1

    def _on_shutdown(self):
        """@brief Close the CSV file on shutdown."""
        try:
            self.csv_file.close()
        except Exception:
            pass


def main():
    """@brief CLI entrypoint for data collection."""
    parser = argparse.ArgumentParser(description="Collect CNN dataset samples")
    parser.add_argument("--output-dir", default="data/collected", help="Output directory")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--pose-topic", default="/pose")
    parser.add_argument("--collision-topic", default="/collision/probability")
    parser.add_argument("--save-rate", type=float, default=5.0, help="Frames per second")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to collect (0 for infinite)")
    parser.add_argument("--max-frames", type=int, default=0, help="Max frames to save (0 for infinite)")
    args = parser.parse_args()

    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    rospy.init_node("cnn_data_collect", anonymous=True)
    DataCollector(
        output_dir,
        args.image_topic,
        args.pose_topic,
        args.collision_topic,
        args.save_rate,
        args.duration,
        args.max_frames,
    )
    rospy.loginfo("Collecting data into %s", output_dir)
    rospy.spin()


if __name__ == "__main__":
    main()
