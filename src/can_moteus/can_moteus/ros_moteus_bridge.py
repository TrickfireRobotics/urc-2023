import rclpy
from rclpy.node import Node
import moteus
import threading

class RosMotuesBridge(Node):

    def __init__(self):
        super().__init__("can_moteus_node")
        self.get_logger().info("Launching can_moteus node")
        

def main(args=None):
    rclpy.init(args=args)
    node = RosMotuesBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()