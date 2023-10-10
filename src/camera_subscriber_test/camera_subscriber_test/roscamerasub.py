import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RosCameraSub(Node):

    def __init__(self):
        super().__init__("ros_camera_sub")
        self.get_logger().info("Launching ros_camera_sub node")
        
        self.subscription = self.create_subscription(
            Image,
            "video_frames",
            self.listerner_callback,
            10
        )
        
        self.subscription
        
        self.br = CvBridge()
        
    def listerner_callback(self, data):
        self.get_logger().info("Getting video frame")
        
        current_frame = self.br.imgmsg_to_cv2(data)
        
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)


    


def main(args=None):
    rclpy.init(args=args)
    node = RosCameraSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

