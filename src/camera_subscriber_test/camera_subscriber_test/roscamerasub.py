import rclpy
from rclpy.node import Node


class RosCameraSub(Node):

    def __init__(self):
        super().__init__("ros_camera_sub")
        self.get_logger().info("Launching ros_camera_sub node")



    


def main(args=None):
    rclpy.init(args=args)
    node = RosCameraSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

