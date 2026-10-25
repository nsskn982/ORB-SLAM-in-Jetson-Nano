#!/usr/bin/env python3
"""
@file map_persistence_node.py
@brief Map persistence node for monocular ORB-SLAM2.

@details
Monitors the ORB-SLAM2 map point cloud (/orb_slam2/map_points) and saves it to
disk as a compressed .npz plus a JSON metadata file, on a timer and via
~save_map / ~load_map services. Enables multi-mission autonomy and
return-to-base workflows. Loading is informational: ORB-SLAM2 itself must be
restarted with map initialization to re-use a saved map.

@author Srikanth Nanda
@date 2026-06-02
"""

import os
import json
import rospy
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class MapPersistenceNode(object):
    """@brief Saves/loads ORB-SLAM point clouds for long-term autonomy."""

    def __init__(self):
        """@brief Initialize parameters, subscriber, services, and save timer."""
        self.map_dir = rospy.get_param("~map_dir", os.path.expanduser("~/jetbot_maps"))
        self.map_name = rospy.get_param("~map_name", "default_map")
        self.save_interval = rospy.get_param("~save_interval", 30.0)  # seconds
        self.load_on_startup = rospy.get_param("~load_on_startup", False)
        
        os.makedirs(self.map_dir, exist_ok=True)
        
        self.last_save_time = rospy.get_time()
        self.point_cloud = None
        self.map_file = os.path.join(self.map_dir, f"{self.map_name}.npz")
        self.metadata_file = os.path.join(self.map_dir, f"{self.map_name}_metadata.json")
        
        # Subscribers
        rospy.Subscriber("/orb_slam2/map_points", PointCloud2, self._on_map_points)
        
        # Service to save/load maps manually
        from std_srvs.srv import Trigger, TriggerResponse
        self._TriggerResponse = TriggerResponse
        rospy.Service("~save_map", Trigger, self._save_map_service)
        rospy.Service("~load_map", Trigger, self._load_map_service)
        
        # Load map on startup if requested
        if self.load_on_startup:
            self._load_map()
        
        # Timer for periodic saving
        rospy.Timer(rospy.Duration(self.save_interval), self._periodic_save)
        
        rospy.loginfo(f"Map persistence node started. Map dir: {self.map_dir}")

    def _on_map_points(self, msg):
        """@brief Cache the ORB-SLAM2 map point cloud.
        @param msg ROS PointCloud2 message.
        """
        try:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if points:
                self.point_cloud = np.array(points, dtype=np.float32)
                rospy.logdebug(f"Received map with {len(self.point_cloud)} points")
        except Exception as e:
            rospy.logwarn(f"Error parsing point cloud: {e}")

    def _periodic_save(self, event):
        """@brief Save the map to disk on a timer if it is large enough.
        @param event ROS Timer event (unused).
        """
        if self.point_cloud is not None and len(self.point_cloud) > 100:
            self._save_map()

    def _save_map(self):
        """@brief Save the current point cloud and metadata to disk."""
        try:
            if self.point_cloud is None:
                rospy.logwarn("No map to save")
                return
            
            # Save point cloud
            np.savez_compressed(self.map_file, points=self.point_cloud)
            
            # Save metadata
            metadata = {
                "timestamp": rospy.get_time(),
                "num_points": len(self.point_cloud),
                "map_name": self.map_name,
            }
            with open(self.metadata_file, "w") as f:
                json.dump(metadata, f, indent=2)
            
            rospy.loginfo(f"Saved map: {self.map_file} ({len(self.point_cloud)} points)")
        except Exception as e:
            rospy.logerr(f"Error saving map: {e}")

    def _load_map(self):
        """@brief Load a saved point cloud from disk (informational).
        @note ORB-SLAM2 must be restarted with map initialization to use it.
        """
        try:
            if not os.path.exists(self.map_file):
                rospy.logwarn(f"Map file not found: {self.map_file}")
                return
            
            data = np.load(self.map_file)
            self.point_cloud = data["points"]
            rospy.loginfo(f"Loaded map: {self.map_file} ({len(self.point_cloud)} points)")
            rospy.logwarn("NOTE: ORB-SLAM2 must be restarted with map initialization")
        except Exception as e:
            rospy.logerr(f"Error loading map: {e}")

    def _save_map_service(self, req):
        """@brief Service handler for a manual map save.
        @param req std_srvs/Trigger request (unused).
        @return std_srvs/TriggerResponse.
        """
        self._save_map()
        return self._TriggerResponse(success=True, message="Map saved")

    def _load_map_service(self, req):
        """@brief Service handler for a manual map load.
        @param req std_srvs/Trigger request (unused).
        @return std_srvs/TriggerResponse.
        """
        self._load_map()
        return self._TriggerResponse(success=True, message="Map loaded")


def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("map_persistence_node", anonymous=False)
    MapPersistenceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
