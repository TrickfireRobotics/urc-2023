"""
Role: This script defines the Autonomous Control Node, responsible for executing rover actuation.

Functionality: Constructs & executes actuation commands using information from the Navigation and 
Decision Making Nodes, handling speed, orientation, and stopping.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - geometry_msgs.msg: Provides the Twist message type used for velocity commands.
"""

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr


class ControlNode(Node):
    """
    A ROS 2 node that publishes velocity commands to control a robot.

    This node allows for controlling the robot's linear and angular velocity by
    publishing `Twist` messages to the "/cmd_vel" topic. It provides methods to send
    velocity commands and logs the commands being sent.

    Attributes:
        cmd_vel_publisher (Publisher): A ROS 2 publisher for the "/cmd_vel" topic.
    """

    def __init__(self) -> None:
        super().__init__("control_node")

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def sendVelocityCommand(self, linear: float, angular: float) -> None:
        """
        Publishes a velocity command with specified linear and angular values.

        Args:
            linear (float): The linear velocity.
            angular (float): The angular velocity.
        """
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Velocity command sent: linear={linear}, angular={angular}")


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the ControlNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """

    rclpy.init(args=args)
    try:
        control_node = ControlNode()
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        control_node.get_logger().info(
            colorStr("Shutting down example_node node", ColorCodes.BLUE_OK)
        )
        control_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
