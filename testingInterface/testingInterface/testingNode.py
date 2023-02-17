import rclpy
from rclpy.node import Node
from /interface/robot_interface #FIGURE OUT HOW TO IMPORT ROBOT INTERFACE


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