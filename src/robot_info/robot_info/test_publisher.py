import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


TOPIC1 = "front_left_drive_motor_velocity_from_robot_interface"


class TestPublisher(Node):
    def __init__(self):
        super().__init__("test_publisher")
        self.get_logger().info("Hello World")

        self.create_timer(0.5, self.callback)

        self.publisher1 = self.create_publisher(Float32, TOPIC1, 10)

    def callback(self):
        msg1 = Float32()
        msg1.data = 0.5
        self.publisher1.publish(msg1)


def main(args=None):
    rclpy.init(args=args)
    publisher = TestPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
