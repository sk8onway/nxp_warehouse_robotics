"""
shelf_goal_planner.py

Goal generation logic for navigating around warehouse shelves.

This module computes robot goal poses relative to detected shelves.
For each shelf, two navigation goals are generated:
1. Object-viewing pose (front side of shelf)
2. QR-viewing pose (back side of shelf)

This separation allows:
- object detection from one side
- QR code scanning from the opposite side
"""

import numpy as np
from typing import List, Tuple, Dict


def generate_shelf_goals(
    shelves: List[Dict],
    offset_distance: float = 1.0
) -> List[Tuple[float, float, float]]:
    """
    Generate navigation goals for each detected shelf.

    For every shelf, two poses are created:
    - One pose in front of the shelf (for object detection)
    - One pose behind the shelf (for QR scanning)

    Each pose is computed along the shelf's orientation (yaw).

    Coordinate logic:
        Given shelf center (x, y) and orientation yaw:
        Forward direction  = +cos(yaw), +sin(yaw)
        Backward direction = -cos(yaw), -sin(yaw)

    Args:
        shelves (List[Dict]): List of detected shelves.
            Each shelf dict must contain:
                - 'center': (x, y) in world coordinates
                - 'yaw': orientation in degrees

        offset_distance (float): Distance (in meters) to place robot
                                 away from the shelf surface.

    Returns:
        List[Tuple[float, float, float]]:
            List of navigation goals as (x, y, yaw)
    """
    shelf_goals = []

    for shelf in shelves:
        # Shelf center in world coordinates
        x, y = shelf['center']

        # Convert shelf orientation from degrees to radians
        yaw = np.radians(shelf['yaw'])

        # --- Object viewing pose (front side of shelf) ---
        obj_x = x + offset_distance * np.cos(yaw)
        obj_y = y + offset_distance * np.sin(yaw)
        obj_pose = (obj_x, obj_y, yaw)

        # --- QR viewing pose (back side of shelf) ---
        qr_x = x - offset_distance * np.cos(yaw)
        qr_y = y - offset_distance * np.sin(yaw)
        qr_pose = (qr_x, qr_y, yaw)

        # Store goals in order: object → QR
        shelf_goals.extend([obj_pose, qr_pose])

    return shelf_goals
