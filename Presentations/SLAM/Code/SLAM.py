#!/usr/bin/env python3
"""
/**
 * @file SLAM.py
 * @brief Minimal implementations of FastSLAM, Active SLAM, MonoSLAM (EKF-SLAM),
 *        and ORB-style feature pipeline for teaching & presentation purposes.
 * @author Srikanth
 * @date 2025
 *
 * This module includes:
 *   - FastSLAM: Rao-Blackwellized particle filter + landmark EKFs
 *   - Active SLAM: Utility-driven action selection (simplified)
 *   - MonoSLAM / EKF-SLAM: Joint state EKF for camera + features
 *   - ORB-SLAM style pipeline: ORB detection, matching, pose estimation
 *
 * Dependencies:
 *   - Python3, OpenCV, NumPy
 */
"""

import numpy as np
import cv2
import math
import random

#################################################################
# 1. FASTSLAM (Simplified RBPF)
#################################################################

class LandmarkEKF:
    """
    /**
     * @class LandmarkEKF
     * @brief Extended Kalman Filter for a single landmark.
     *
     * This class maintains the mean and covariance of one landmark
     * and provides an EKF update step.
     */
    """
    def __init__(self, mu, Sigma):
        """/**
        * @brief Construct a new LandmarkEKF object.
        * @param mu Initial mean (2D vector)
        * @param Sigma Initial covariance (2x2 matrix)
        */"""
        self.mu = mu
        self.Sigma = Sigma

    def update(self, z, H, R):
        """/**
        * @brief Perform EKF measurement update.
        * @param z Measurement vector
        * @param H Observation Jacobian
        * @param R Measurement noise covariance
        */"""
        S = H @ self.Sigma @ H.T + R
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ (z - H @ self.mu)
        self.Sigma = (np.eye(len(self.mu)) - K @ H) @ self.Sigma

class FastSLAMParticle:
    """
    /**
     * @class FastSLAMParticle
     * @brief One particle in the Rao Blackwellized particle filter.
     *
     * Contains robot pose and a map of landmark EKFs.
     */
    """
    def __init__(self, pose):
        """/**
        * @brief Construct a new FastSLAMParticle with an initial pose.
        * @param pose 3D vector [x, y, theta]
        */"""
        self.pose = pose
        self.landmarks = {}

    def predict(self, u, noise):
        """/**
        * @brief Apply motion model with noise.
        * @param u Control input [v, w]
        * @param noise Noise standard deviations [nv, nw]
        */"""
        v, w = u
        noisy_v = v + np.random.normal(0, noise[0])
        noisy_w = w + np.random.normal(0, noise[1])

        x, y, th = self.pose
        x += noisy_v * math.cos(th)
        y += noisy_v * math.sin(th)
        th += noisy_w
        self.pose = np.array([x, y, th])

class FastSLAM:
    """
    /**
     * @class FastSLAM
     * @brief Rao Blackwellized SLAM with independent landmark EKFs.
     */
    """
    def __init__(self, num_particles=50):
        """/**
        * @brief Construct FastSLAM with given number of particles.
        * @param num_particles Number of particles
        */"""
        self.particles = [FastSLAMParticle(np.zeros(3)) for _ in range(num_particles)]

    def predict(self, u):
        """/**
        * @brief Predict step for all particles.
        * @param u Control input [v, w]
        */"""
        for p in self.particles:
            p.predict(u, noise=[0.01, 0.01])

    def update(self, z, landmark_id):
        """/**
        * @brief Update all particles with a measurement.
        * @param z Measurement of landmark position
        * @param landmark_id ID of the observed landmark
        */"""
        for p in self.particles:
            if landmark_id not in p.landmarks:
                p.landmarks[landmark_id] = LandmarkEKF(mu=np.array([z[0], z[1]]),
                                                       Sigma=np.eye(2) * 1.0)

            lm = p.landmarks[landmark_id]
            H = np.eye(2)
            R = np.eye(2) * 0.05
            lm.update(z, H, R)

#################################################################
# 2. ACTIVE SLAM (Simplified Utility-Based Planning)
#################################################################

class ActiveSLAM:
    """
    /**
     * @class ActiveSLAM
     * @brief Simplified utility-driven SLAM planner.
     */
    """
    def __init__(self):
        """/**
        * @brief Construct ActiveSLAM with predefined action set.
        */"""
        self.actions = [
            np.array([0.2, 0.0]),   # forward
            np.array([0.0, 0.4]),   # rotate left
            np.array([0.0, -0.4])   # rotate right
        ]

    def info_gain(self, Sigma):
        """/**
        * @brief Compute expected information gain.
        * @param Sigma Current covariance
        * @return Log determinant score
        */"""
        return -np.log(np.linalg.det(Sigma) + 1e-9)

    def choose_action(self, Sigma):
        """/**
        * @brief Choose action that maximizes information gain.
        * @param Sigma Current covariance
        * @return Selected control action
        */"""
        best = None
        best_u = None
        for u in self.actions:
            pred_sigma = Sigma - np.eye(2) * 0.001
            score = self.info_gain(pred_sigma)
            if best is None or score > best:
                best = score
                best_u = u
        return best_u

#################################################################
# 3. Monocular EKF-SLAM (MonoSLAM-like)
#################################################################

class EKFSLAM:
    """
    /**
     * @class EKFSLAM
     * @brief MonoSLAM-like EKF joint state estimator.
     */
    """
    def __init__(self, max_landmarks=20):
        """/**
        * @brief Construct EKF-SLAM.
        * @param max_landmarks Maximum number of landmarks
        */"""
        self.mu = np.zeros(3)
        self.Sigma = np.eye(3) * 0.01
        self.max_landmarks = max_landmarks
        self.landmarks = {}

    def predict(self, u):
        """/**
        * @brief EKF predict step.
        * @param u Control input [v, w]
        */"""
        v, w = u
        x, y, th = self.mu

        x_new = x + v * math.cos(th)
        y_new = y + v * math.sin(th)
        th_new = th + w

        self.mu = np.array([x_new, y_new, th_new])
        self.Sigma += np.eye(3) * 0.001

    def update(self, landmark_id, z):
        """/**
        * @brief EKF update with a landmark measurement.
        * @param landmark_id ID of the observed landmark
        * @param z 2D measurement vector
        */"""
        if landmark_id not in self.landmarks:
            self.landmarks[landmark_id] = LandmarkEKF(mu=np.array(z),
                                                      Sigma=np.eye(2) * 1.0)
        self.landmarks[landmark_id].update(
            z=z,
            H=np.eye(2),
            R=np.eye(2) * 0.05
        )

#################################################################
# 4. ORB-SLAM-style Feature Detection + Matching
#################################################################

class ORB_SLAM_Pipeline:
    """
    /**
     * @class ORB_SLAM_Pipeline
     * @brief Simplified ORB front-end pipeline.
     */
    """
    def __init__(self):
        """/**
        * @brief Construct ORB-SLAM pipeline with ORB extractor and BF matcher.
        */"""
        self.orb = cv2.ORB_create(2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def extract(self, frame):
        """/**
        * @brief Extract ORB keypoints and descriptors.
        * @param frame Input image
        * @return (keypoints, descriptors)
        */"""
        kp, des = self.orb.detectAndCompute(frame, None)
        return kp, des

    def match(self, des1, des2):
        """/**
        * @brief Match descriptors using KNN + Lowe ratio test.
        * @param des1 Descriptors from image 1
        * @param des2 Descriptors from image 2
        * @return List of good matches
        */"""
        raw = self.matcher.knnMatch(des1, des2, k=2)
        good = []
        for m, n in raw:
            if m.distance < 0.75 * n.distance:
                good.append(m)
        return good

    def estimate_pose(self, kp1, kp2, matches):
        """/**
        * @brief Estimate pose using matched keypoints and homography.
        * @param kp1 Keypoints from image 1
        * @param kp2 Keypoints from image 2
        * @param matches Filtered matches
        * @return Homography matrix or None
        */"""
        if len(matches) < 8:
            return None

        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])

        H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC)
        return H
