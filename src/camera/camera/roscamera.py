import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from rclpy.executors import MultiThreadedExecutor
import cv2 # OpenCV library
from pathlib import Path
from std_msgs.msg import String

# returns a list of working camera ids to capture every camera connected to the robot
def get_cameras() -> list[int]:
    nonWorkingPorts = 0
    devPort = 0
    workingPorts = []

    while nonWorkingPorts < 6:
        camera = cv2.VideoCapture(devPort)
        if not camera.isOpened():
            nonWorkingPorts += 1
        else:
            isReading, img = camera.read()
            _ = camera.get(3)
            _ = camera.get(4)
            if isReading:
                workingPorts.append(devPort)
        
        devPort += 1
    
    return workingPorts

class RosCamera(Node):

    def __init__(self, topicName: str, camera: int):
        super().__init__("ros_camera")
        self.get_logger().info("Launching ros_camera node")

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self._publisher = self.create_publisher(CompressedImage, topicName, 10)
        self.get_logger().info("Created Publisher " + topicName)

        # Create a subscription to read 'camera_control' topic from
        # mission control. Topic messages are Strings.
        #       ex. topic message: "toggle arm_camera, ID: 0"
        # The topic message should always have the corresponding camera number
        # at the last index of each message.
        subTopicName = "camera_control"
        self.subscription = self.create_subscription(
            String,
            subTopicName,
            self.toggle_camera,
            10)
        self.subscription
        self.get_logger().info("Created Subscription to" + subTopicName)

        # We will publish a message every 0.1 seconds
        # timer_period = 0.1  # seconds
        timer_period = 2  # seconds
        
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(camera)
        self.get_logger().info("Using video ID: " + str(camera) + ", ON: " + str(self.cap.isOpened()))
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        # Declare parameters for each 'ros_camera' node thread
        self.declare_parameter("cameraNumber", camera)
        self.declare_parameter("isOn", True)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()
            
        # Get 'ros_camera' parameter, isOn, then check the state of the camera to publish frame or not
        isOn = bool(self.get_parameter("isOn")._value)
        if ret and isOn:
            # Publish the image.
            self._publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))
    
        # Display the message on the console
        # self.get_logger().info('Publishing compressed video frame')
        
    def toggle_camera(self, msg) -> None:
        # Read the last index of the message and check whether mission control is targeting this
        # instance of 'ros_camera'
        cameraNumber = int(self.get_parameter("cameraNumber")._value)
        if (int(msg.data[-1]) != cameraNumber):
            return

        # Toggle by not'ing the bool value
        isOn = bool(self.get_parameter("isOn")._value)
        self.undeclare_parameter("isOn")
        self.declare_parameter("isOn", not isOn)

def main(args=None):
    rclpy.init(args=args)
    try:
        """
        we need an executor because running .spin() is a blocking function.
        using the MultiThreadedExecutor, we can control multiple nodes
        """
        executor = MultiThreadedExecutor()
        nodes = []
        cameraNum = 0

        for cameraID in get_cameras():
            node = RosCamera("video_frames" + str(cameraNum), cameraID)
            nodes.append(node)
            executor.add_node(node)
            cameraNum += 1
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
