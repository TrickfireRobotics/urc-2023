"""
Role: This script defines the sensor processing node, responsible for processing all sensor data, 
primarily camera images, for object and obstacle detection. Data is published in a format suitable 
for other nodes.

Functionality:

- Subscribe to images and process for obstacle & object recognition with OpenCV.
- Outputs data to the Decision Making and Localization Nodes as needed.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - sensor_msgs.msg: Provides LaserScan and Imu for sensor data.
    - std_msgs.msg: Provides Float32 for processed sensor data.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray
from typing import Optional

import numpy as np
import cv2  # pylint: disable=no-member
from cv_bridge import CvBridge
from cv2 import aruco

from lib.color_codes import ColorCodes, colorStr


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data.

    This node subscribes to raw sensor data, processes it, and publishes the
    processed results to other nodes for use in decision-making and control.

    Attributes:
        image_sub (Subscription): ROS 2 subscription for the camera image topic.
        camera_info_sub (Subscription): ROS 2 subscription for the camera info topic.
        aruco_pub (Publisher): Publishes ArUco marker data (ID and position).
        lidar_sub (Subscription): ROS 2 subscription for the lidar data.
        imu_sub (Subscription): ROS 2 subscription for IMU data.
        processed_pub (Publisher): Publishes processed sensor data (example: min lidar distance).
        bridge (CvBridge): For converting ROS images to OpenCV images.
        camera_matrix (Optional[np.ndarray]): Camera intrinsic matrix from /camera_info.
        dist_coeffs (Optional[np.ndarray]): Distortion coefficients from /camera_info.
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")
        self.get_logger().info("Initializing sensor_processing_node...")

        # Subscribers for ZED 2i camera and calibration data
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.processCameraImage, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # Publisher for detected ArUco marker data (Float32MultiArray)
        self.aruco_pub = self.create_publisher(Float32MultiArray, "/aruco_marker_data", 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Camera intrinsic parameters (will be set from /camera_info)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # Subscribers for raw sensor data
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu_data", self.processImu, 10)

        # Publisher for processed data
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

        self.get_logger().info("sensor_processing_node is up and running, waiting for data...")

    def processCameraInfo(self, msg: CameraInfo) -> None:
        """
        Processes camera intrinsic parameters from /camera_info topic.

        Args:
            msg (CameraInfo): Camera intrinsic parameters message.
        """
        self.get_logger().info("Received camera_info message. Updating intrinsics.")
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.get_logger().info("Camera calibration parameters have been set.")

    def processCameraImage(self, msg: Image) -> None:
        """
        Processes the incoming camera image to detect ArUco markers.
        Also draws markers and axes onto the image and displays the result.

        Args:
            msg (Image): ROS2 image message from the ZED 2i.
        """
        self.get_logger().debug("processCameraImage callback triggered.")
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Initialize ArUco dictionary and detector
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Detect ArUco markers in the grayscale image
        corners, ids, rejected = aruco_detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            # Example: if you see "[17]", that means one marker with ID=17 was detected
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

            # Estimate pose for each detected marker
            # NOTE: Adjust tag_size to the actual size of your marker in meters
            tag_size = 0.2
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, tag_size, self.camera_matrix, self.dist_coeffs
            )

            # Draw the detected markers on the original image
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                pos_x, pos_y, pos_z = tvecs[i][0]

                # Draw the coordinate axes on the marker (for visualization)
                aruco.drawAxis(
                    cv_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    0.1,  # axis length in meters
                )

                # Publish marker data to /aruco_marker_data
                marker_msg = Float32MultiArray()
                marker_msg.data = [marker_id, pos_x, pos_y, pos_z]
                self.aruco_pub.publish(marker_msg)

                self.get_logger().info(
                    f"Marker {marker_id}: Position -> x={pos_x:.2f}, y={pos_y:.2f}, z={pos_z:.2f}"
                )

        else:
            # Log that no markers were found for this frame
            self.get_logger().debug("No ArUco markers detected in this frame.")

        # Display the image with drawn markers (requires a valid GUI environment)
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def processLidar(self, msg: LaserScan) -> None:
        """
        Processes lidar data and publishes a simple result (min distance).

        Args:
            msg (LaserScan): The raw lidar data.
        """
        self.get_logger().debug("processLidar callback triggered.")
        if not msg.ranges:
            self.get_logger().warning("Lidar ranges data is empty.")
            return

        # Example: Calculate the minimum distance
        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance:.2f}m")
        self.processed_pub.publish(Float32(data=min_distance))

    def processImu(self, msg: Imu) -> None:
        """
        Processes IMU data and logs orientation.

        Args:
            msg (Imu): The raw IMU data.
        """
        self.get_logger().debug("processImu callback triggered.")
        orientation = msg.orientation
        self.get_logger().info(
            "Processed IMU data. Orientation: "
            f"x={orientation.x:.3f}, y={orientation.y:.3f}, "
            f"z={orientation.z:.3f}, w={orientation.w:.3f}"
        )


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the SensorProcessingNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    sensor_processing_node = SensorProcessingNode()

    try:
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        sensor_processing_node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except ExternalShutdownException:
        sensor_processing_node.get_logger().info(
            colorStr(
                "External shutdown requested. Shutting down sensor_processing_node",
                ColorCodes.BLUE_OK,
            )
        )
    finally:
        # Clean up OpenCV windows and destroy the node
        cv2.destroyAllWindows()
        sensor_processing_node.destroy_node()
        rclpy.shutdown()
        # Removed any direct call to sys.exit(0) so it won’t forcibly exit on detection.


if __name__ == "__main__":
    main()
