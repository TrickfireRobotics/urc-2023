""" Module containing all the code for Autonomous Control """

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node


class ControlNode(Node):
    """Control Node"""

    def __init__(self) -> None:
        super().__init__("control_node")

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def sendVelocityCommand(self, linear: Vector3, angular: Vector3) -> None:
        """Publishes a velocity command with specified linear and angular values"""

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Velocity command sent: linear={linear}, angular={angular}")


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    control_node = ControlNode()

    # Example command to move forward with a slight turn
    control_node.sendVelocityCommand(0.5, 0.1)

    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
