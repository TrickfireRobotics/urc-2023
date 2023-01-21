import rclpy
from rclpy.node import Node


class RosMotuesBridge(Node):
    def __init__(self):
        super().__init__("can_motues_node")

        

        self.get_logger().info("Hello from can_motues_node from RosMotuesBridge __init__")


def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
