"""
Node to convert /cmd_vel (Twist) to individual wheel velocities for differential drive.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class CmdVelToWheelVel(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_wheel_vel")

        # Parameters
        self.declare_parameter("wheel_base", 0.5)  # Distance between wheels in meters
        self.wheel_base = self.get_parameter("wheel_base").value

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publishers
        self.left_wheel_pub = self.create_publisher(Float32, "/left_wheel_velocity", 10)
        self.right_wheel_pub = self.create_publisher(Float32, "/right_wheel_velocity", 10)

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Publish
        left_msg = Float32()
        left_msg.data = float(left_vel)
        self.left_wheel_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = float(right_vel)
        self.right_wheel_pub.publish(right_msg)

        self.get_logger().debug(f"Left vel: {left_vel}, Right vel: {right_vel}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheelVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
