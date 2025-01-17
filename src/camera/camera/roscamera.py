import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from std_msgs.msg import String


def getCameras() -> list[int]:
    """
    Function returns a list of working camera IDs to capture every camera connected to the robot
    """

    non_working_ports = 0
    dev_port = 0
    working_ports = []

    while non_working_ports < 6:
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports += 1
        else:
            is_reading, img = camera.read()
            _ = camera.get(3)
            _ = camera.get(4)
            if is_reading:
                working_ports.append(dev_port)

        dev_port += 1

    return working_ports


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
        #        ex. topic message: "0 off", "0 on"
        # The topic message should always have the corresponding camera number
        # at the first index of each message.
        sub_topic_name = "camera_control"
        self.subscription = self.create_subscription(
            String, sub_topic_name, self.setCameraState, 10
        )
        self.get_logger().info("Created Subscription to" + sub_topic_name)

        # We will publish a message every 0.1 seconds
        # timer_period = 0.1  # seconds
        timer_period = 0.1  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.publishCameraFrame)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(camera)
        self.get_logger().info(
            "Using video ID: " + str(camera) + ", ON: " + str(self.cap.isOpened())
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Declare parameters for each 'ros_camera' node thread
        self.declare_parameter("cameraNumber", camera)
        self.declare_parameter("isOn", True)

    def publishCameraFrame(self) -> None:
        """
        Callback function publishes a frame captured from a camera to /video_framesX (X is specific
        camera ID) every 0.1 seconds
        """

        # Capture frame-by-frame
        # This method returns True/False as well as the video frame.
        ret, frame = self.cap.read()

        # Get bool parameter, isOn, and check the state of the camera to publish frame or not
        # Value of isOn is controlled by setCameraState()
        is_on = bool(self.get_parameter("isOn")._value)
        if ret and is_on:
            # Publish the image.
            self._publisher.publish(self.br.cv2_to_compressed_imgmsg(frame))

    def setCameraState(self, msg: String) -> None:
        """
        Function reads str msgs from topic, 'camera_control' (published by mission control), and
        turns on/off the periodic publishing of frames by a certain camera

        Function is called whenever a msg is published to 'camera_control'

        Function assumes str msgs sent to 'camera_control' begin with a number and ends with an 'f'
        (meaning off) or 'n' (meaning on)
            ex.
                "0 off" -> to turn off camera with id=0
                "2 on" -> to turn on camera with id=2
        """

        # Check if this specific camera is targeted by mission control, stop the function otherwise
        camera_number = int(self.get_parameter("cameraNumber")._value)
        if int(msg.data[0]) != camera_number:
            return

        # Check if the desired camera state is already fulfilled, print and stop the function if so
        camera_on = bool(self.get_parameter("isOn")._value)
        if str(msg.data[-1]) == "f" and not camera_on:
            self.get_logger().info("camera" + str(camera_number) + " is already off!")
            return
        elif str(msg.data[-1]) == "n" and camera_on:
            self.get_logger().info("camera" + str(camera_number) + " is already on!")
            return

        # Set isOn bool parameter to either stop or restart the publishing of frames
        self.undeclare_parameter("isOn")
        if str(msg.data[-1]) == "f":
            # self.get_logger().info("turning camera" + str(cameraNumber) + " off!")
            self.declare_parameter("isOn", False)
        else:
            # self.get_logger().info("turning camera" + str(cameraNumber) + " on!")
            self.declare_parameter("isOn", True)


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        # We need an executor because running .spin() is a blocking function.
        # using the MultiThreadedExecutor, we can control multiple nodes
        executor = MultiThreadedExecutor()
        nodes = []
        camera_num = 0

        for camera_id in getCameras():
            node = RosCamera("video_frames" + str(camera_num), camera_id)
            nodes.append(node)
            executor.add_node(node)
            camera_num += 1

        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in nodes:
                node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
