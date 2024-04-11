import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


class RosCamera(Node):

    def __init__(self):
        super().__init__("ros_camera")
        self.get_logger().info("Launching ros_camera node")

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(CompressedImage, 'video_frames', 10)

        # We will publish a message every 0.1 seconds
        # timer_period = 0.1  # seconds
        timer_period = 2  # seconds
        
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()
            
        if ret == True:
            # Publish the image.
            self.publisher_.publish(self.br.cv2_to_compressed_imgmsg(frame))
    
        # Display the message on the console
        #self.get_logger().info('Publishing compressed video frame')
        



    


def main(args=None):
    rclpy.init(args=args)
    node = RosCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
