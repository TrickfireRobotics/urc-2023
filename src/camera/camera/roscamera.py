import rclpy
from rclpy.node import Node


class RosCamera(Node):

    def __init__(self):
        super().__init__("ros_camera")
        self.get_logger().info("Launching ros_camera node")



    


def main(args=None):
    rclpy.init(args=args)
    node = RosCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

