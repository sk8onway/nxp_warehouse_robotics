"""
frontier_exploration.py

This module implements frontier-based exploration logic.

Responsibilities:
- Identify frontiers from a global occupancy grid
- Select the best frontier to explore next
- Decide when exploration is complete
- Trigger transition from exploration mode to navigation mode

This module does NOT create a ROS node.
It is designed to be used inside an existing ROS2 node.
"""

import time
import numpy as np
from scipy.spatial.distance import euclidean


class FrontierExplorer:
    def __init__(self, node):
        """
        Initialize frontier exploration handler.

        Args:
            node: Reference to the parent ROS2 node.
        """
        self.node = node

        # Store latest global map
        self.global_map_curr = None

        # Timestamp of last detected frontier
        self.last_frontier_timestamp = time.time()

        # Time threshold after which exploration is considered complete
        self.exploration_timeout_sec = 10

        # Exploration distance constraints (in world coordinates)
        self.max_step_dist_world_meters = 7.0
        self.min_step_dist_world_meters = 4.0

        # Counter to track repeated empty frontier detections
        self.full_map_explored_count = 0

    def global_map_callback(self, message):
        """
        Callback to process global map updates and perform frontier exploration.

        Args:
            message: nav_msgs.msg.OccupancyGrid
        """

        self.global_map_curr = message

        # Do not explore if a navigation goal is currently active
        if not self.node.goal_completed:
            return

        height = message.info.height
        width = message.info.width

        # Convert 1D occupancy data into 2D array
        map_array = np.array(message.data).reshape((height, width))

        # Detect frontiers
        frontiers = self.get_frontiers(map_array)
        self.node.get_logger().info(
            f"[Exploration] Frontiers found: {len(frontiers)}"
        )

        # ----- EXPLORATION COMPLETION CHECK -----
        if len(frontiers) == 0:
            # No frontiers found → possible exploration completion
            if time.time() - self.last_frontier_timestamp > self.exploration_timeout_sec:
                if not self.node.navigation_mode:
                    self.node.activate_navigation_mode()
            else:
                self.node.get_logger().info(
                    "Waiting before switching to navigation mode..."
                )
            return

        # Frontiers exist → update timestamp
        self.last_frontier_timestamp = time.time()

        # ----- FRONTIER SELECTION -----
        map_info = message.info
        closest_frontier = None
        min_distance = float("inf")

        for fy, fx in frontiers:
            fx_world, fy_world = self.node.get_world_coord_from_map_coord(
                fx, fy, map_info
            )

            distance = euclidean(
                (fx_world, fy_world),
                self.node.buggy_center
            )

            # Apply distance constraints
            if (
                distance < min_distance and
                self.min_step_dist_world_meters <= distance <= self.max_step_dist_world_meters
            ):
                min_distance = distance
                closest_frontier = (fy, fx)

        # ----- SEND EXPLORATION GOAL -----
        if closest_frontier:
            fy, fx = closest_frontier
            goal = self.node.create_goal_from_map_coord(fx, fy, map_info)

            self.node.send_goal_from_world_pose(goal)

            self.node.get_logger().info(
                f"[Exploration] Closest frontier selected at map coords: {closest_frontier}"
            )
            self.node.get_logger().info(
                "[Exploration] Sending goal for space exploration."
            )

        else:
            # No valid frontier within distance thresholds
            self.node.get_logger().warn(
                "[Exploration] No valid frontier found. Expanding search range."
            )
            self.max_step_dist_world_meters += 2.0
            self.min_step_dist_world_meters = max(
                0.25,
                self.min_step_dist_world_meters - 1.0
            )

            self.full_map_explored_count = 0

    def get_frontiers(self, map_array):
        """
        Identify frontier cells in the occupancy grid.

        A frontier cell is:
        - Unknown (-1)
        - Adjacent to at least one free cell (0)
        - Not adjacent to obstacles

        Args:
            map_array: 2D numpy array of occupancy values

        Returns:
            List of (y, x) frontier coordinates
        """
        frontiers = []

        for y in range(1, map_array.shape[0] - 1):
            for x in range(1, map_array.shape[1] - 1):

                # Only consider unknown cells
                if map_array[y, x] != -1:
                    continue

                # Check surrounding cells for obstacles
                neighbors_all = [
                    (y, x - 1), (y, x + 1),
                    (y - 1, x), (y + 1, x),
                    (y - 1, x - 1), (y + 1, x - 1),
                    (y - 1, x + 1), (y + 1, x + 1)
                ]

                if any(map_array[ny, nx] > 0 for ny, nx in neighbors_all):
                    continue  # Too close to obstacle

                # Check if adjacent to free space
                neighbors_cardinal = [
                    (y, x - 1), (y, x + 1),
                    (y - 1, x), (y + 1, x)
                ]

                if any(map_array[ny, nx] == 0 for ny, nx in neighbors_cardinal):
                    frontiers.append((y, x))

        return frontiers
