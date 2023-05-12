import rclpy
from rclpy.node import Node
import sys
sys.path.insert(1, '/home/trickfire/urc-2023/src/interface')

class testingNode(Node):

    def __init__(self):
        super().__init__("testing_node")
        self.get_logger().info("Launching testing_node node")
        robotInterface = robotInterface(self)


def main(args=None):
    rclpy.init(args=args)
    node = testingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
