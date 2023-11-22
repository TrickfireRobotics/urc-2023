import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


TOPIC1 = "mid_right_drive_motor_velocity"
TOPIC2 = "mid_left_drive_motor_velocity"


class TestPublisher(Node):
    def __init__(self):
        super().__init__("test_publisher")

        self.create_timer(0.5, self.callback)

        self.publisher1 = self.create_publisher(Float32, TOPIC1, 10)
        self.publisher2 = self.create_publisher(Float32, TOPIC2, 10)

    def callback(self):
        msg1 = Float32()
        msg1.data = random.random() * 100
        self.publisher1.publish(msg1)
        self.get_logger().info(f"Sent topic: {TOPIC1}; data: {msg1.data}")

        msg2 = Float32()
        msg2.data = random.random() * 100
        self.publisher2.publish(msg2)
        self.get_logger().info(f"Sent topic: {TOPIC2}; data: {msg2.data}")


def main(args=None):
    rclpy.init(args=args)
    publisher = TestPublisher()
    rclpy.spin(publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
