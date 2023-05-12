import sys
import rclpy
from rclpy.node import Node
import time

sys.path.append('/home/trickfire/urc-2023/src/interface')
from robot_interface import RobotInterface



class testingNode(Node):

    def __init__(self):
        super().__init__("testing_node")
        self.get_logger().info("Launching testing_node node")
        robotInterface = RobotInterface(self)
        time.sleep(3)

        robotInterface.front_left_drive_motor(0.1)
        robotInterface.front_right_drive_motor(0.1 * 12)
        robotInterface.mid_left_drive_motor(0.1 * 12)


def main(args=None):
    rclpy.init(args=args)
    node = testingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
