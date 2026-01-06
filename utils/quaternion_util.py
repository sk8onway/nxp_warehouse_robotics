"""
quaternion_utils.py

Utility functions for working with quaternions in 2D navigation contexts.

In mobile robotics (Nav2, SLAM, AMCL), robot orientation is typically
represented as a quaternion even though motion is planar (yaw only).
This file provides helpers to convert yaw (rotation about Z-axis)
into a geometry_msgs Quaternion.
"""

import math
from geometry_msgs.msg import Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """
    Creates a quaternion representing a rotation about the Z-axis (yaw).

    This function assumes:
    - 2D planar motion (no roll or pitch)
    - Rotation is only around the Z-axis (standard for ground robots)

    Mathematical background:
    A quaternion for a yaw-only rotation is:
        qx = 0
        qy = 0
        qz = sin(yaw / 2)
        qw = cos(yaw / 2)

    Args:
        yaw (float): Rotation about the Z-axis in radians.

    Returns:
        Quaternion: geometry_msgs Quaternion representing the orientation.
    """
    half_yaw = yaw * 0.5

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)

    return q
