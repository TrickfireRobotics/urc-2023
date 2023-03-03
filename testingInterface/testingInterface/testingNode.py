import rclpy
from rclpy.node import Node
import sys
sys.path.append('urc-2023/src/interface/src/robot_interface.py')
import robot_interface.py


class testingNode(Node):

    def init(self):
        super().init("testing_node")
        self.get_logger().info("Launching testing_node node")
        robotInterface = robotInterface(self)


def main(args=None):
    rclpy.init(args=args)
    node = testingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if name == "main":
    main()