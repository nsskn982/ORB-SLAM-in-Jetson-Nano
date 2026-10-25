#!/usr/bin/env python3
"""
@file decision_node.py
@brief ROS node that generates cmd_vel based on pose and collision probability.

@details
Reactive motion controller for the SparkFun JetBot v2.1-based mobile robot.
It consumes the monocular ORB-SLAM2 pose (/pose) and a collision probability
(/collision/probability, from the map-based or CNN detector) and produces a
rate-limited TwistStamped command on /cmd_vel. The default behaviour is purely
reactive: drive forward, hold the current heading, slow down on moderate
collision risk, and perform a spin/back-off recovery on high risk or stale pose.

@note Goal-seeking is intentionally NOT enabled in this version. To extend the
      node toward waypoint navigation, see the GOAL-SEEKING SCAFFOLDING block in
      __init__ and _control_callback below. The hooks are written but commented
      out so the reactive baseline is unchanged.

@author Srikanth Nanda
@date 2026-06-02
"""

import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import Float32
from jetbot_utils import quaternion_to_yaw, normalize_angle, clamp, as_bool


class DecisionNode(object):
    def __init__(self):
        """@brief Initialize node state, ROS interfaces, and timers."""
        self.pose_topic = rospy.get_param("~pose_topic", "/pose")
        self.collision_topic = rospy.get_param("~collision_topic", "/collision/probability")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.queue_size = int(rospy.get_param("~queue_size", 1))
        self.control_hz = float(rospy.get_param("~control_hz", 10.0))

        self.linear_speed = float(rospy.get_param("~linear_speed", 0.12))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.80))

        self.collision_stop_threshold = float(rospy.get_param("~collision_stop_threshold", 0.60))
        self.collision_slow_threshold = float(rospy.get_param("~collision_slow_threshold", 0.35))
        self.collision_timeout = float(rospy.get_param("~collision_timeout", 1.0))

        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.50))
        self.heading_kp = float(rospy.get_param("~heading_kp", 1.20))
        self.max_heading_correction = float(rospy.get_param("~max_heading_correction", 0.50))

        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.15))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.00))

        self.recovery_flip_period = float(rospy.get_param("~recovery_flip_period", 2.0))
        self.recovery_timeout = float(rospy.get_param("~recovery_timeout", 2.5))
        self.allow_reverse = as_bool(rospy.get_param("~allow_reverse", True))
        self.reverse_speed = abs(float(rospy.get_param("~reverse_speed", 0.05)))
        self.reverse_timeout = float(rospy.get_param("~reverse_timeout", 1.0))

        alpha = float(rospy.get_param("~angular_smoothing_alpha", 0.35))
        # BUGFIX (2026-06-02): use the imported clamp() from jetbot_utils.
        # The previous self._clamp(...) call raised AttributeError at startup
        # because _clamp was never defined as an instance method.
        self.angular_smoothing_alpha = clamp(alpha, 0.0, 1.0)

        # ------------------------------------------------------------------
        # GOAL-SEEKING SCAFFOLDING (disabled by default)
        # ------------------------------------------------------------------
        # To turn this reactive controller into a goal/waypoint navigator,
        # uncomment the block below and the matching block in
        # _control_callback(). The robot would then steer its heading toward
        # the goal bearing while still respecting the collision behaviour.
        #
        # self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        # self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.15))  # m
        # self.goal_pose = None  # geometry_msgs/Pose, set by _goal_callback
        # self.goal_sub = rospy.Subscriber(
        #     self.goal_topic, PoseStamped, self._goal_callback,
        #     queue_size=self.queue_size,
        # )
        # ------------------------------------------------------------------

        self.current_collision_prob = 1.0
        self.current_yaw = 0.0
        self.target_yaw = None
        self.last_pose = None  # cached PoseStamped.pose for goal scaffolding
        self.filtered_angular_z = 0.0

        self.last_pose_time = rospy.Time(0)
        self.last_collision_time = rospy.Time(0)
        self.recovery_turn_direction = 1.0
        self.last_recovery_flip = rospy.Time.now()
        self.recovery_start_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped, queue_size=self.queue_size)

        self.pose_sub = rospy.Subscriber(
            self.pose_topic,
            PoseStamped,
            self._pose_callback,
            queue_size=self.queue_size,
        )
        self.collision_sub = rospy.Subscriber(
            self.collision_topic,
            Float32,
            self._collision_callback,
            queue_size=self.queue_size,
        )

        period = rospy.Duration(1.0 / max(1.0, self.control_hz))
        self.timer = rospy.Timer(period, self._control_callback)

        rospy.loginfo(
            "decision_node initialized pose_topic=%s collision_topic=%s cmd_vel_topic=%s",
            self.pose_topic,
            self.collision_topic,
            self.cmd_vel_topic,
        )

    def _collision_callback(self, msg):
        """@brief Update current collision probability.
        @param msg ROS Float32 message.
        """
        self.current_collision_prob = clamp(float(msg.data), 0.0, 1.0)
        self.last_collision_time = rospy.Time.now()

    def _pose_callback(self, msg):
        """@brief Update the latest pose and yaw estimate.
        @param msg ROS PoseStamped message.
        """
        q = msg.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        # Cached for the optional goal-seeking scaffolding (position lookup).
        self.last_pose = msg.pose

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw

        if msg.header.stamp.to_sec() > 0.0:
            self.last_pose_time = msg.header.stamp
        else:
            self.last_pose_time = rospy.Time.now()

    def _control_callback(self, _event):
        """@brief Timer callback that computes and publishes cmd_vel.
        @param _event ROS Timer event (unused).
        """
        now = rospy.Time.now()
        
        # Check if collision data is stale
        collision_fresh = (now - self.last_collision_time).to_sec() <= self.collision_timeout
        if not collision_fresh:
            rospy.logwarn_throttle(5, "Collision data stale, stopping robot")
            cmd = Twist()
            self._publish_cmd(cmd)
            return

        if self.current_collision_prob >= self.collision_stop_threshold:
            cmd = self._build_recovery_cmd(now, spin_scale=1.0)
            self._publish_cmd(cmd)
            return

        if self.current_collision_prob >= self.collision_slow_threshold:
            self._reset_recovery_state()
            cmd = Twist()
            cmd.linear.x = self.linear_speed * 0.5
            cmd.angular.z = self.recovery_turn_direction * self.turn_speed * 0.35
            self._publish_cmd(cmd)
            return

        pose_fresh = (
            self.target_yaw is not None and
            (now - self.last_pose_time).to_sec() <= self.pose_timeout
        )

        if not pose_fresh:
            cmd = self._build_recovery_cmd(now, spin_scale=0.35)
            self._publish_cmd(cmd)
            return

        self._reset_recovery_state()
        cmd = Twist()

        # GOAL-SEEKING SCAFFOLDING (disabled by default):
        # When enabled, replace target_yaw with the live goal bearing so the
        # robot turns toward the goal instead of merely holding its heading.
        #   goal_yaw = self._goal_bearing()
        #   if goal_yaw is not None:
        #       self.target_yaw = goal_yaw

        yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
        heading_correction = clamp(
            self.heading_kp * yaw_error,
            -self.max_heading_correction,
            self.max_heading_correction,
        )

        cmd.linear.x = self.linear_speed
        if abs(yaw_error) > 0.6:
            cmd.linear.x *= 0.6
        cmd.angular.z = heading_correction

        self._publish_cmd(cmd)

    # ----------------------------------------------------------------------
    # GOAL-SEEKING SCAFFOLDING (disabled by default)
    # ----------------------------------------------------------------------
    # def _goal_callback(self, msg):
    #     """@brief Store the latest navigation goal.
    #     @param msg ROS PoseStamped goal in the map frame.
    #     """
    #     self.goal_pose = msg.pose
    #
    # def _goal_bearing(self):
    #     """@brief Compute the desired heading toward the current goal.
    #     @return Target yaw (rad) in the map frame, or None if no goal/pose.
    #     """
    #     if self.goal_pose is None or self.last_pose is None:
    #         return None
    #     dx = self.goal_pose.position.x - self.last_pose.position.x
    #     dy = self.goal_pose.position.y - self.last_pose.position.y
    #     if (dx * dx + dy * dy) ** 0.5 <= self.goal_tolerance:
    #         return None  # goal reached; fall back to reactive hold
    #     return math.atan2(dy, dx)
    # ----------------------------------------------------------------------

    def _build_recovery_cmd(self, now, spin_scale):
        """@brief Build a recovery Twist when pose is stale or collision risk is high.
        @param now Current ROS time.
        @param spin_scale Scaling for angular speed.
        @return Twist command.
        """
        if self.recovery_start_time.to_sec() <= 0.0:
            self.recovery_start_time = now
            self.last_recovery_flip = now

        if (now - self.last_recovery_flip).to_sec() >= self.recovery_flip_period:
            self.recovery_turn_direction *= -1.0
            self.last_recovery_flip = now

        elapsed = (now - self.recovery_start_time).to_sec()
        cmd = Twist()

        if elapsed <= self.recovery_timeout:
            cmd.linear.x = 0.0
            cmd.angular.z = self.recovery_turn_direction * self.turn_speed * spin_scale
        elif self.allow_reverse and elapsed <= (self.recovery_timeout + self.reverse_timeout):
            cmd.linear.x = -self.reverse_speed
            cmd.angular.z = self.recovery_turn_direction * self.turn_speed * 0.20
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        return cmd

    def _reset_recovery_state(self):
        """@brief Reset recovery state timers."""
        self.recovery_start_time = rospy.Time(0)

    def _publish_cmd(self, cmd):
        """@brief Clamp and publish the velocity command.
        @param cmd Twist command to publish.
        """
        cmd.linear.x = clamp(cmd.linear.x, -self.max_linear_speed, self.max_linear_speed)

        target_angular = clamp(cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.filtered_angular_z = (
            self.angular_smoothing_alpha * target_angular +
            (1.0 - self.angular_smoothing_alpha) * self.filtered_angular_z
        )
        cmd.angular.z = clamp(self.filtered_angular_z, -self.max_angular_speed, self.max_angular_speed)

        # Publish with timestamp
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist = cmd
        self.cmd_pub.publish(twist_stamped)




def main():
    """@brief ROS node entrypoint."""
    rospy.init_node("decision_node")
    DecisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
