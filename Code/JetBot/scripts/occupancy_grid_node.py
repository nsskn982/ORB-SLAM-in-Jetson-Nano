#!/usr/bin/env python3
"""
@file occupancy_grid_node.py
@brief Convert ORB-SLAM2 map points into a ROS OccupancyGrid for collision detection.

@details
Subscribes to the monocular ORB-SLAM2 sparse point cloud (/orb_slam2/map_points)
and pose (/pose) and publishes a local, world-axis-aligned OccupancyGrid (/map)
centred on the robot. Points are filtered by height, projected directly into a
world-aligned grid (no heading rotation), inflated by a configurable radius, and
published at a fixed rate. The map origin orientation is identity (yaw = 0), and
map_collision_node.py consumes this same world-aligned convention.

@note Design choice: the grid is WORLD-AXIS-ALIGNED, not heading-aligned. The grid
      x/y axes stay parallel to the SLAM world frame regardless of robot yaw. This
      keeps the grid consistent with a future global/persistent map and matches the
      origin_yaw = 0 assumption in map_collision_node._world_to_cell().

@author Srikanth Nanda
@date 2026-06-02
"""

import math
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class OccupancyGridNode(object):
    """@brief Builds a world-aligned local OccupancyGrid from SLAM map points."""

    def __init__(self):
        """@brief Initialize parameters, state, publishers, subscribers, and timer."""
        # Grid parameters
        self.grid_width = int(rospy.get_param("~grid_width", 20))         # cells
        self.grid_height = int(rospy.get_param("~grid_height", 20))       # cells
        self.grid_resolution = float(rospy.get_param("~grid_resolution", 0.05))  # m/cell
        self.max_height = float(rospy.get_param("~max_height", 0.5))      # ignore points above (m)
        self.min_height = float(rospy.get_param("~min_height", -0.1))     # ignore points below (m)
        self.inflation_radius = float(rospy.get_param("~inflation_radius", 0.15))  # m
        self.publish_freq = float(rospy.get_param("~publish_freq", 2.0))  # Hz

        # State
        self.point_cloud = None
        self.robot_pose = None
        self.lock = threading.Lock()

        # Publisher (latched so late subscribers get the most recent grid)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber("/orb_slam2/map_points", PointCloud2, self._on_map_points)
        rospy.Subscriber("/pose", PoseStamped, self._on_pose)  # from ORB-SLAM2

        # Timer for periodic grid updates
        period = rospy.Duration(1.0 / max(self.publish_freq, 1e-3))
        rospy.Timer(period, self._update_grid)

        rospy.loginfo(
            "occupancy_grid_node started. Grid: %dx%d, resolution: %.3f m/cell",
            self.grid_width,
            self.grid_height,
            self.grid_resolution,
        )

    def _on_map_points(self, msg):
        """@brief Receive and cache the SLAM map point cloud.
        @param msg ROS PointCloud2 message of map points.
        """
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if points:
                with self.lock:
                    self.point_cloud = np.array(points, dtype=np.float32)
        except Exception as exc:
            rospy.logwarn("Error parsing point cloud: %s", str(exc))

    def _on_pose(self, msg):
        """@brief Receive and cache the robot pose from ORB-SLAM2.
        @param msg ROS PoseStamped message.
        """
        with self.lock:
            self.robot_pose = msg.pose

    def _update_grid(self, _event):
        """@brief Build and publish the occupancy grid from cached data.
        @param _event ROS Timer event (unused).
        """
        with self.lock:
            if self.point_cloud is None or self.robot_pose is None:
                return
            points = self.point_cloud.copy()
            robot_pose = self.robot_pose

        # Filter points by height band (z) to ignore floor/ceiling clutter.
        points = points[
            (points[:, 2] >= self.min_height) &
            (points[:, 2] <= self.max_height)
        ]
        if len(points) == 0:
            rospy.logwarn_throttle(5.0, "No points in valid height range")
            return

        # Create empty grid initialised to unknown (-1).
        grid_data = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        # Robot position in the world frame (grid centre).
        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y

        # World-axis-aligned projection: no heading rotation is applied.
        for point in points:
            dx = point[0] - robot_x
            dy = point[1] - robot_y

            grid_x = int(dx / self.grid_resolution + self.grid_width / 2.0)
            grid_y = int(dy / self.grid_resolution + self.grid_height / 2.0)

            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                grid_data[grid_y, grid_x] = 100  # occupied

        # Inflate obstacles by the configured radius.
        grid_data = self._inflate_obstacles(grid_data)

        # Build and publish the OccupancyGrid message.
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.info.map_load_time = rospy.Time.now()
        msg.info.resolution = self.grid_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # Lower-left origin in world frame; identity orientation (yaw = 0)
        # so the grid stays world-axis-aligned (see module docstring).
        msg.info.origin.position.x = robot_x - self.grid_width * self.grid_resolution / 2.0
        msg.info.origin.position.y = robot_y - self.grid_height * self.grid_resolution / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid_data.flatten().tolist()  # row-major
        self.map_pub.publish(msg)

        rospy.logdebug("Published occupancy grid (%d occupied cells)", int(np.sum(grid_data > 50)))

    def _inflate_obstacles(self, grid):
        """@brief Inflate occupied cells by inflation_radius to add safety margin.
        @param grid 2D int8 occupancy array.
        @return Inflated 2D int8 occupancy array.
        """
        inflated = grid.copy()
        inflation_cells = int(self.inflation_radius / self.grid_resolution)
        if inflation_cells <= 0:
            return inflated

        occupied = grid > 50
        for i in range(grid.shape[0]):
            for j in range(grid.shape[1]):
                if occupied[i, j]:
                    y_min = max(0, i - inflation_cells)
                    y_max = min(grid.shape[0], i + inflation_cells + 1)
                    x_min = max(0, j - inflation_cells)
                    x_max = min(grid.shape[1], j + inflation_cells + 1)

                    inflated[y_min:y_max, x_min:x_max] = np.maximum(
                        inflated[y_min:y_max, x_min:x_max],
                        50,  # semi-occupied
                    )
        return inflated


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("occupancy_grid_node", anonymous=False)
    OccupancyGridNode()
    rospy.spin()


if __name__ == "__main__":
    main()
