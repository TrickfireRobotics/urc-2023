import rclpy
from rclpy.node import Node

class Heartbeat(Node):
    def __init__(self):
        super().__init__("heartbeat_node")
        self.get_logger().info("Hi from heartbeat!")

def main(args=None):
    rclpy.init(args=args)
    node = Heartbeat()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    