import sys
import rclpy
from rclpy.node import Node
import time

sys.path.append('/home/trickfire/urc-2023/src/interface')
from robot_interface import RobotInterface



class TestingNode(Node):

    def __init__(self):
        super().__init__("testing_node")
        self.get_logger().info("Launching testing_node node")
        robotInterface = RobotInterface(self)
        


def main(args=None):
    rclpy.init(args=args)
    node = TestingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
