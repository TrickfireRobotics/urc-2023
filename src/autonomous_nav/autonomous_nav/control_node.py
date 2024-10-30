import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def send_velocity_command(self, linear, angular):
        # Publishes a velocity command with specified linear and angular values
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(
            f"Velocity command sent: linear={linear}, angular={angular}"
        )


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()

    # Example command to move forward with a slight turn
    control_node.send_velocity_command(0.5, 0.1)

    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
