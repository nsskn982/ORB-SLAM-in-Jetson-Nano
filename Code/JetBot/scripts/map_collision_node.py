#!/usr/bin/env python3
"""
@file map_collision_node.py
@brief Map-based collision avoidance using an occupancy grid.
@author Srikanth Nanda
@date 2026-05-24
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32
from jetbot_utils import quaternion_to_yaw, as_bool


class MapCollisionNode(object):
    def __init__(self):
        """@brief Initialize parameters, subscribers, and publishers."""
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.pose_topic = rospy.get_param("~pose_topic", "/pose")

        self.inference_hz = float(rospy.get_param("~inference_hz", 5.0))
        self.queue_size = int(rospy.get_param("~queue_size", 1))

        self.forward_distance = float(rospy.get_param("~forward_distance", 0.8))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.25))
        self.robot_width = float(rospy.get_param("~robot_width", 0.30))
        self.sample_step = float(rospy.get_param("~sample_step", 0.05))

        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 50))
        self.treat_unknown_as_obstacle = as_bool(
            rospy.get_param("~treat_unknown_as_obstacle", True)
        )

        self.collision_threshold = float(rospy.get_param("~collision_threshold", 0.60))
        self.fail_safe_probability = float(rospy.get_param("~fail_safe_probability", 1.0))
        self.map_timeout = float(rospy.get_param("~map_timeout", 2.0))  # seconds
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))  # seconds

        self.map_lock = threading.Lock()
        self.map_data = None
        self.map_info = None
        self.map_origin_yaw = 0.0
        self.last_map_time = rospy.Time.now()

        self.pose_lock = threading.Lock()
        self.pose = None
        self.last_pose_time = rospy.Time.now()

        self.prob_pub = rospy.Publisher("/collision/probability", Float32, queue_size=self.queue_size)
        self.risky_pub = rospy.Publisher("/collision/risky", Bool, queue_size=self.queue_size)

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_callback, queue_size=1)

        period = rospy.Duration(1.0 / max(self.inference_hz, 1e-3))
        self.timer = rospy.Timer(period, self._timer_callback)

        rospy.loginfo("map_collision_node initialized map_topic=%s pose_topic=%s", self.map_topic, self.pose_topic)

    def _map_callback(self, msg):
        """@brief Store the latest occupancy grid.
        @param msg ROS OccupancyGrid message.
        """
        with self.map_lock:
            self.map_data = msg.data
            self.map_info = msg.info
            q = msg.info.origin.orientation
            self.map_origin_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.last_map_time = rospy.Time.now()

    def _pose_callback(self, msg):
        """@brief Store the latest pose.
        @param msg ROS PoseStamped message.
        """
        with self.pose_lock:
            self.pose = msg.pose
            self.last_pose_time = rospy.Time.now()

    def _timer_callback(self, _event):
        """@brief Periodic collision probability update.
        @param _event ROS timer event.
        """
        probability = self._compute_collision_probability()
        self._publish_probability(probability)

    def _compute_collision_probability(self):
        """@brief Compute collision probability from the occupancy grid.
        @return Probability in [0, 1].
        """
        now = rospy.Time.now()
        
        with self.map_lock:
            map_data = self.map_data
            map_info = self.map_info
            origin_yaw = self.map_origin_yaw
            map_age = (now - self.last_map_time).to_sec()

        with self.pose_lock:
            pose = self.pose
            pose_age = (now - self.last_pose_time).to_sec()

        # Check timeouts
        if map_age > self.map_timeout:
            rospy.logwarn_throttle(5, f"Map data stale ({map_age:.1f}s > {self.map_timeout}s)")
            return self.fail_safe_probability
        
        if pose_age > self.pose_timeout:
            rospy.logwarn_throttle(5, f"Pose data stale ({pose_age:.1f}s > {self.pose_timeout}s)")
            return self.fail_safe_probability

        if map_data is None or map_info is None or pose is None:
            return self.fail_safe_probability

        if map_info.resolution <= 0.0 or map_info.width == 0 or map_info.height == 0:
            return self.fail_safe_probability

        heading = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        samples = 0
        occupied = 0
        min_obstacle_dist = None

        dist = 0.0
        while dist <= self.forward_distance:
            lateral = -0.5 * self.robot_width
            while lateral <= 0.5 * self.robot_width:
                world_x, world_y = self._project_point(pose.position.x, pose.position.y, heading, dist, lateral)
                cell = self._world_to_cell(world_x, world_y, map_info, origin_yaw)
                if cell is None:
                    if self.treat_unknown_as_obstacle:
                        occupied += 1
                        samples += 1
                    lateral += self.sample_step
                    continue

                value = map_data[cell]
                if value < 0:
                    if self.treat_unknown_as_obstacle:
                        occupied += 1
                        samples += 1
                    lateral += self.sample_step
                    continue

                samples += 1
                if value >= self.occupied_threshold:
                    occupied += 1
                    if min_obstacle_dist is None or dist < min_obstacle_dist:
                        min_obstacle_dist = dist

                lateral += self.sample_step

            dist += self.sample_step

        if samples == 0:
            return self.fail_safe_probability

        if min_obstacle_dist is not None and min_obstacle_dist <= self.stop_distance:
            return 1.0

        return float(occupied) / float(samples)

    def _publish_probability(self, probability):
        """@brief Publish collision probability and risky flag.
        @param probability Probability value.
        """
        prob = max(0.0, min(1.0, float(probability)))
        self.prob_pub.publish(Float32(data=prob))
        self.risky_pub.publish(Bool(data=(prob >= self.collision_threshold)))

    def _world_to_cell(self, x, y, info, origin_yaw):
        """@brief Convert world coordinates to map cell index.
        @param x World X.
        @param y World Y.
        @param info OccupancyGrid info.
        @param origin_yaw Map origin yaw.
        @return Linear cell index or None if out of bounds.
        """
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y

        cos_yaw = math.cos(-origin_yaw)
        sin_yaw = math.sin(-origin_yaw)
        mx = (dx * cos_yaw - dy * sin_yaw) / info.resolution
        my = (dx * sin_yaw + dy * cos_yaw) / info.resolution

        ix = int(math.floor(mx))
        iy = int(math.floor(my))

        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return None
        return iy * info.width + ix

    def _project_point(self, x, y, heading, forward, lateral):
        """@brief Project a point in robot frame into the world frame.
        @param x Robot world X.
        @param y Robot world Y.
        @param heading Robot heading yaw.
        @param forward Forward distance.
        @param lateral Lateral offset.
        @return Tuple of world coordinates.
        """
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        world_x = x + forward * cos_h - lateral * sin_h
        world_y = y + forward * sin_h + lateral * cos_h
        return world_x, world_y



def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("map_collision_node")
    MapCollisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
