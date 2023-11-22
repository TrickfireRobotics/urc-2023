import rclpy
from rclpy.node import Node

from .robot_info import RobotInfo


class TestSubscriber(Node):
    def __init__(self):
        super().__init__("test_subscriber")
        self.info = RobotInfo(self, self.callback)

    def callback(self, topic, data):
        self.get_logger().info(f"received topic: {topic}; data: {data}")
        self.get_logger().info("info.mid_left_drive_motor_velocity: "
                               + str(self.info.mid_left_drive_motor_velocity))
        self.get_logger().info("info.mid_right_drive_motor_velocity: "
                               + str(self.info.mid_right_drive_motor_velocity))


def main(args=None):
    rclpy.init(args=args)
    subscriber = TestSubscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
