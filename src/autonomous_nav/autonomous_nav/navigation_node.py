"""
Role: This script defines the navigation node responsible for managing the navigation logic of the 
rover.

Functionality: The node calculates paths to a goal location and sends movement commands 
to the robot based on the calculated paths. Receives information from the Decision Making Node

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - geometry_msgs.msg: Provides PoseStamped for goal positions.
    - std_msgs.msg: Provides String for control commands.
"""

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr


class NavigationNode(Node):
    """
    A ROS 2 node for handling robot navigation.

    This node subscribes to goal positions and publishes movement commands to
    guide the robot to its target location.

    Attributes:
        goal_sub (Subscription): A ROS 2 subscription for the "/goal_pose" topic.
        control_pub (Publisher): A ROS 2 publisher for the "/control_commands" topic.
        current_goal (PoseStamped): Tracks the current goal position of the robot.
    """

    def __init__(self) -> None:
        super().__init__("navigation_node")

        # Subscriber for the goal position
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.processGoal, 10)

        # Publisher for control commands
        self.control_pub = self.create_publisher(String, "/control_commands", 10)

        # Initialize the current goal
        self.current_goal = PoseStamped()

    def processGoal(self, msg: PoseStamped) -> None:
        """
        Processes the received goal position and generates a movement plan.

        Args:
            msg (PoseStamped): The goal position for the robot.
        """
        self.current_goal = msg
        self.get_logger().info(f"Received goal: {msg.pose.position.x}, {msg.pose.position.y}")
        self.control_pub.publish(String(data="MOVE_TO_GOAL"))

    def stopNavigation(self) -> None:
        """
        Publishes a stop command to halt the robot's navigation.
        """
        self.control_pub.publish(String(data="STOP"))
        self.get_logger().info("Navigation stopped.")


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the NavigationNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    try:
        navigation_node = NavigationNode()
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        navigation_node.get_logger().info(
            colorStr("Shutting down example_node node", ColorCodes.BLUE_OK)
        )
        navigation_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
