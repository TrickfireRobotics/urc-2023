import sys
import rclpy
from rclpy.node import Node

sys.path.append("/home/trickfire/urc-2023/src")

from std_msgs.msg import String
from robot_info.robot_info import RobotInfo


class MissionControlData(Node):

    def __init__(self):
        super().__init__('mission_control_data_node')
        self.get_logger().info("Creating mission control data node")
        self.info = RobotInfo(self, self.callback)

    def callback(self, topic, data):
        self.get_logger().info(f"received topic: {topic}; data: {data}")
        self.get_logger().info("info.mid_left_drive_motor_velocity: "
                               + str(self.info.mid_left_drive_motor_velocity))

    #     self.publisher_ = self.create_publisher(String, 'topic', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0
    #     self.get_logger().info("Launching mission_control_data node")
    #     self.subscription = self.create_subscription(
    #         String,
    #         'topic',
    #         self.listener_callback,
    #         10
    #     )
    #     self.subscription

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    mission_control_data_node = MissionControlData()
    rclpy.spin(mission_control_data_node)
    mission_control_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
