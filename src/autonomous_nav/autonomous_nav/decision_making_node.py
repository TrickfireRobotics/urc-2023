"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality: Receives inputs from Navigation, Sensor Processing, and Localization Nodes to 
make decisions on stopping, rerouting, or changing navigation.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - std_msgs.msg: Provides the String and Bool message types used for communication.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from lib.color_codes import ColorCodes, colorStr


class DecisionMakingNode(Node):
    """
    A ROS 2 node that handles decision-making for a robot.

    This node subscribes to obstacle detection and target tracking topics, and
    publishes control commands based on the received data. It ensures the robot
    reacts appropriately to environmental changes and maintains its tasks.

    Attributes:
        control_pub (Publisher): A ROS 2 publisher for the "/control_commands" topic.
        obstacle_detected (bool): Tracks whether an obstacle is detected.
        target_lost (bool): Tracks whether the target has been lost.
    """

    def __init__(self) -> None:
        super().__init__("decision_making_node")

        # Publisher to send commands to Control Node
        self.control_pub = self.create_publisher(String, "/control_commands", 10)

        # Subscribers to listen for input from other nodes
        self.create_subscription(Bool, "/obstacle_detected", self.collisionHandling, 10)
        self.create_subscription(Bool, "/target_lost", self.targetReacquisition, 10)

        # Initialize state variables
        self.obstacle_detected = False
        self.target_lost = False

    def collisionHandling(self, msg: Bool) -> None:
        """
        Handles obstacle detection and decides on stopping or rerouting.

        Args:
            msg (Bool): A message indicating whether an obstacle is detected.
        """
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            self.get_logger().info("Obstacle detected! Stopping rover.")
            self.control_pub.publish(String(data="STOP"))
        else:
            self.get_logger().info("Path clear, resuming navigation.")
            self.control_pub.publish(String(data="RESUME"))

    def targetReacquisition(self, msg: Bool) -> None:
        """
        Handles reacquiring the target if lost and reorienting if needed.

        Args:
            msg (Bool): A message indicating whether the target is lost.
        """
        self.target_lost = msg.data
        if self.target_lost:
            self.get_logger().info("Target lost, initiating search.")
            self.control_pub.publish(String(data="SEARCH_FOR_TARGET"))
        else:
            self.get_logger().info("Target reacquired, resuming tracking.")
            self.control_pub.publish(String(data="RESUME_TRACKING"))


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the DecisionMakingNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    try:
        decision_making_node = DecisionMakingNode()
        rclpy.spin(decision_making_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        decision_making_node.get_logger().info(
            colorStr("Shutting down example_node node", ColorCodes.BLUE_OK)
        )
        decision_making_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
