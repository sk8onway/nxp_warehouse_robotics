#!/usr/bin/env python3
"""
main_node.py

Central orchestrator node for the NXP AIM India Warehouse Robotics Project.

Responsibilities:
- Maintain global FSM (Exploration → Shelf Navigation)
- Own ROS subscriptions and publishers
- Delegate work to perception, exploration, and navigation modules
- Handle Nav2 goal lifecycle (send → feedback → result)
"""

import rclpy
from rclpy.node import Node
from enum import Enum

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# ---------------- MODULES ----------------
from navigation.frontier_exploration import FrontierExplorer
from navigation.navigation_controller import NavigationController
from navigation.shelf_goal_planner import generate_shelf_goals
from perception.shelf_detection_opencv import detect_shelves

from utils.map_utils import get_world_coord_from_map_coord
from utils.quaternion_util import quaternion_from_yaw


# ---------------- FSM ----------------
class RobotState(Enum):
    EXPLORATION = 1
    NAVIGATION_TO_SHELVES = 2


class WarehouseMainNode(Node):

    def __init__(self):
        super().__init__("warehouse_main_node")

        # ---------------- FSM STATE ----------------
        self.state = RobotState.EXPLORATION

        # ---------------- ROBOT STATE ----------------
        self.current_pose = None
        self.global_map = None

        # ---------------- NAVIGATION STATE ----------------
        self.shelf_goals = []
        self.current_goal_index = 0
        self.goal_active = False

        # ---------------- NAV2 CLIENT ----------------
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        # ---------------- MODULES ----------------
        self.frontier_explorer = FrontierExplorer(self)
        self.navigator = NavigationController(self)

        # ---------------- SUBSCRIPTIONS ----------------
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.global_map_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_callback,
            10
        )

        self.get_logger().info("Warehouse Main Node initialized")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update robot pose."""
        self.current_pose = msg.pose.pose

    def global_map_callback(self, msg: OccupancyGrid):
        """Main map-driven logic."""
        self.global_map = msg

        if self.state == RobotState.EXPLORATION:
            self.frontier_explorer.global_map_callback(msg)

        elif self.state == RobotState.NAVIGATION_TO_SHELVES:
            if not self.shelf_goals:
                self.prepare_and_start_shelf_navigation()

    # ============================================================
    # EXPLORATION → NAVIGATION TRANSITION
    # ============================================================

    def activate_navigation_mode(self):
        """Called by FrontierExplorer when exploration is complete."""
        if self.state == RobotState.NAVIGATION_TO_SHELVES:
            return

        self.get_logger().info("Exploration complete → Switching to shelf navigation")
        self.state = RobotState.NAVIGATION_TO_SHELVES

    # ============================================================
    # SHELF NAVIGATION PIPELINE
    # ============================================================

    def prepare_and_start_shelf_navigation(self):
        """Detect shelves, generate goals, and start Nav2 navigation."""
        self.get_logger().info("📦 Detecting shelves from global map")

        # Convert OccupancyGrid to grayscale image
        width = self.global_map.info.width
        height = self.global_map.info.height
        map_array = list(self.global_map.data)
        map_np = (255 - (255 * (map_array > 0))).reshape((height, width)).astype("uint8")

        # Detect shelves (pixel coords)
        shelves_px = detect_shelves(map_np)

        if not shelves_px:
            self.get_logger().warn("No shelves detected")
            return

        # Convert to world coordinates
        shelves_world = []
        for shelf in shelves_px:
            mx, my = shelf["map_center"]
            wx, wy = get_world_coord_from_map_coord(
                int(mx), int(my), self.global_map.info
            )

            shelves_world.append({
                "center": (wx, wy),
                "yaw": shelf["yaw"]
            })

        # Generate navigation goals
        self.shelf_goals = []
        raw_goals = generate_shelf_goals(shelves_world)

        for x, y, yaw in raw_goals:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = quaternion_from_yaw(yaw)

            self.shelf_goals.append(pose)

        self.current_goal_index = 0
        self.send_next_shelf_goal()

    def send_next_shelf_goal(self):
        """Send the next shelf goal to Nav2."""
        if self.current_goal_index >= len(self.shelf_goals):
            self.get_logger().info("All shelf goals completed")
            return

        if self.goal_active:
            return

        self.goal_active = True
        goal_pose = self.shelf_goals[self.current_goal_index]
        self.navigator.send_goal(goal_pose)

    # ============================================================
    # NAV2 CALLBACKS
    # ============================================================

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected")
            self.goal_active = False
            return

        self.get_logger().info("Nav2 goal accepted")
        self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        self.goal_active = False

        if status == 4:  # SUCCEEDED
            self.get_logger().info("✅ Goal reached")
            self.current_goal_index += 1
        else:
            self.get_logger().warn(f"Goal failed (status={status})")

        self.send_next_shelf_goal()

    def nav_feedback_callback(self, feedback_msg):
        pass  # optional


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
