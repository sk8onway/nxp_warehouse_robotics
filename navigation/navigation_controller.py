"""
navigation_controller.py

This module handles high-level navigation control for visiting detected shelves.
It is responsible for:
- Switching the robot from exploration mode to navigation mode
- Sending navigation goals sequentially to Nav2
- Managing the goal queue for shelf inspection

This file does NOT create a new ROS node.
It is intended to be used as part of an existing node (e.g., warehouse_explore).
"""

from geometry_msgs.msg import PoseStamped


class NavigationController:
    def __init__(self, node):
        """
        Initialize the navigation controller.

        Args:
            node: Reference to the parent ROS2 node.
                  Used for logging, time, and Nav2 action client access.
        """
        self.node = node

        # List of navigation goals to visit (x, y, yaw)
        self.shelf_goals = []

        # Index of the currently active goal in the goal list
        self.current_goal_index = 0

        # Flag to indicate whether robot has switched to navigation mode
        self.navigation_mode = False

    def send_next_shelf_goal(self):
        """
        Sends the next navigation goal from the shelf_goals list.

        This function:
        - Checks if all shelf goals are completed
        - Constructs a PoseStamped message
        - Sends the goal to Nav2 (action client call is currently commented)
        - Advances the goal index

        This function assumes Nav2 is already running and ready.
        """

        # Check if all goals have already been sent
        if self.current_goal_index >= len(self.shelf_goals):
            self.node.get_logger().info("All shelf goals completed.")
            return

        # Extract the next goal (x, y, yaw)
        goal = self.shelf_goals[self.current_goal_index]
        x, y, yaw = goal

        # Create a PoseStamped message for Nav2
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Nav2 expects goals in map frame
        pose.header.stamp = self.node.get_clock().now().to_msg()

        # Set target position
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw (heading) to quaternion orientation
        pose.pose.orientation = self.node._create_quaternion_from_yaw(yaw)

        # Log for debugging / visibility
        self.node.get_logger().info(
            f"Sending navigation goal to: ({x:.2f}, {y:.2f})"
        )

        # Send goal to Nav2 (intentionally commented for safety/testing)
        # self.node.nav_to_pose_client.send_goal(pose)

        # Move to the next goal for the next call
        self.current_goal_index += 1

    def activate_navigation_mode(self):
        """
        Switches the robot from exploration mode to navigation mode.

        This function:
        - Enables navigation mode
        - Resets the goal index
        - Prepares the system to start visiting shelf goals

        Typically triggered once exploration is complete
        (e.g., no frontiers found for N seconds).
        """

        self.navigation_mode = True
        self.current_goal_index = 0

        self.node.get_logger().info("Switched to Navigation Mode.")
