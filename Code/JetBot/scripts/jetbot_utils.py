#!/usr/bin/env python3
"""
Shared utilities for JetBot nodes.
Eliminates code duplication across multiple nodes.
"""

import math


def quaternion_to_yaw(x, y, z, w):
    """
    Convert quaternion to yaw angle in radians.
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        Yaw angle in radians [-pi, pi]
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """
    Wrap angle to [-pi, pi] range.
    
    Args:
        angle: Input angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value, low, high):
    """
    Clamp a value to range [low, high].
    
    Args:
        value: Value to clamp
        low: Minimum value
        high: Maximum value
        
    Returns:
        Clamped value
    """
    return max(low, min(high, value))


def as_bool(value):
    """
    Convert common truthy values to boolean.
    
    Args:
        value: Input value
        
    Returns:
        Boolean interpretation
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() in ("1", "true", "yes", "on")
