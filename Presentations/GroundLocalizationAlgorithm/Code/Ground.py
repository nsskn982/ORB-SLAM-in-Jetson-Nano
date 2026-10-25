#!/usr/bin/env python3
"""
/**
 * @file Ground.py
 * @brief Ground Texture Localization Algorithms: Feature-Based, Appearance-Based,
 *        Visual Odometry, and Map-Based Localization.
 * @author Srikanth
 * @date 2025
 *
 * This module provides implementations of different ground-texture-based
 * localization algorithms using OpenCV:
 *
 *  - Feature-Based Ground Texture Localization
 *  - Appearance-Based Localization (Correlation)
 *  - Feature-Based Visual Odometry
 *  - Map-Based Absolute Localization
 *
 * Requires: OpenCV, NumPy
 */
"""

import cv2
import numpy as np

############################################################
# 1. FEATURE-BASED GROUND TEXTURE LOCALIZATION
############################################################

def feature_based_localization(img1, img2, method="ORB"):
    """
    Detect & match local features between two ground texture images.
    Args:
        img1: reference image
        img2: query image
        method: 'SIFT', 'ORB', 'AKAZE'
    Returns:
        matches, kp1, kp2
    """
    if method == "SIFT":
        detector = cv2.SIFT_create()
        norm = cv2.NORM_L2
    elif method == "AKAZE":
        detector = cv2.AKAZE_create()
        norm = cv2.NORM_HAMMING
    else:
        detector = cv2.ORB_create(nfeatures=1000)
        norm = cv2.NORM_HAMMING

    kp1, des1 = detector.detectAndCompute(img1, None)
    kp2, des2 = detector.detectAndCompute(img2, None)

    matcher = cv2.BFMatcher(norm)
    raw_matches = matcher.knnMatch(des1, des2, k=2)

    # Lowe ratio test
    good = []
    for m, n in raw_matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)

    return kp1, kp2, good


############################################################
# 2. APPEARANCE-BASED LOCALIZATION (PATCH CORRELATION)
############################################################

def appearance_based_localization(ref, query, patch_size=32, search_radius=32):
    """
    Compute displacement between two frames using template matching.
    Uses NCC (Normalized Cross Correlation).
    """
    h, w = ref.shape[:2]
    cx, cy = w // 2, h // 2

    # Extract central reference patch
    ref_patch = ref[cy - patch_size:cy + patch_size,
                    cx - patch_size:cx + patch_size]

    search = query[cy - search_radius:cy + search_radius,
                   cx - search_radius:cx + search_radius]

    res = cv2.matchTemplate(search, ref_patch, cv2.TM_CCOEFF_NORMED)
    _, _, _, max_loc = cv2.minMaxLoc(res)

    dx = max_loc[0] - (search_radius - patch_size)
    dy = max_loc[1] - (search_radius - patch_size)

    return dx, dy


############################################################
# 3. FEATURE-BASED VISUAL ODOMETRY
############################################################

def feature_based_vo(prev_frame, curr_frame):
    """
    Estimate frame-to-frame motion using feature matching + RANSAC.
    """
    orb = cv2.ORB_create(1500)
    kp1, des1 = orb.detectAndCompute(prev_frame, None)
    kp2, des2 = orb.detectAndCompute(curr_frame, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    raw = bf.knnMatch(des1, des2, k=2)

    good = []
    for m, n in raw:
        if m.distance < 0.75 * n.distance:
            good.append(m)

    if len(good) < 8:
        return None, None, None

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])

    H, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC)

    return H, kp1, kp2


############################################################
# 4. MAP-BASED ABSOLUTE LOCALIZATION
############################################################

def map_based_localization(query, map_keypoints, map_descriptors):
    """
    Query a global map of ground keypoints using FLANN.
    """
    detector = cv2.SIFT_create()
    kpq, desq = detector.detectAndCompute(query, None)

    index_params = dict(algorithm=1, trees=5)   # KD-tree
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    raw_matches = flann.knnMatch(desq, map_descriptors, k=2)

    good = []
    for m, n in raw_matches:
        if m.distance < 0.75 * n.distance:
            good.append(m)

    if len(good) < 8:
        return None, kpq, []

    pts_q = np.float32([kpq[m.queryIdx].pt for m in good])
    pts_m = np.float32([map_keypoints[m.trainIdx].pt for m in good])

    H, mask = cv2.findHomography(pts_q, pts_m, cv2.RANSAC)

    return H, kpq, good