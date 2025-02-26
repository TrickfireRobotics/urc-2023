"""
Role: This script defines the sensor processing node, responsible for processing all sensor data, 
primarily camera images, for object and obstacle detection. Data is published in a format suitable 
for other nodes.

Functionality:
- Subscribe to images and process for obstacle & object recognition with OpenCV.
- Outputs data to the Decision Making and GPS Anchor Nodes as needed.
- Detects large, close obstacles via the ZED stereo depth map.
- Publishes obstacle data for the DecisionMakingNode to use.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray, Bool
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
        camera_info_sub (Subscription): ROS 2 subscription for camera intrinsics.
        aruco_pub (Publisher): Publishes ArUco marker data (ID and position).
        lidar_sub (Subscription): ROS 2 subscription for the lidar data.
        processed_pub (Publisher): Publishes processed sensor data (example: min lidar distance).
        bridge (CvBridge): For converting ROS images to OpenCV images.
        camera_matrix (Optional[np.ndarray]): Camera intrinsic matrix from /camera_info.
        dist_coeffs (Optional[np.ndarray]): Distortion coefficients from /camera_info.

        depth_sub (Subscription): ROS 2 subscription for the depth image (stereo camera).
        obstacle_detected_pub (Publisher): Publishes a Bool indicating if a large obstacle is detected.
        obstacle_info_pub (Publisher): Publishes Float32MultiArray with obstacle details.
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")
        self.get_logger().info("Initializing sensor_processing_node...")

        # -------------------------------------------------
        #   ZED 2i camera data (color + intrinsics)
        # -------------------------------------------------
        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # -------------------------------------------------
        #   ArUco marker detection publisher
        # -------------------------------------------------
        self.aruco_pub = self.create_publisher(Float32MultiArray, "/aruco_marker_data", 10)

        # -------------------------------------------------
        #   LIDAR data
        # -------------------------------------------------
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

        # -------------------------------------------------
        #   Depth-based obstacle detection
        # -------------------------------------------------
        # Subscribe to the ZED node's depth image (adjust topic name if needed)
        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.obstacleDetection, 10
        )
        # Publishers for obstacle detection
        self.obstacle_detected_pub = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.obstacle_info_pub = self.create_publisher(Float32MultiArray, "/obstacle_info", 10)

        self.get_logger().info("sensor_processing_node is up and running, waiting for data...")

    # --------------------------------------------------------------------------
    #   Camera / Intrinsics
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        """
        Processes camera intrinsic parameters from /camera_info topic.

        Args:
            msg (CameraInfo): Camera intrinsic parameters message.
        """
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Processes the incoming camera image to detect ArUco markers.
        Also draws markers and axes onto the image and displays the result.

        Args:
            msg (Image): ROS2 image message from the ZED 2i.
        """
        self.get_logger().debug("arucoMarkerDetection callback triggered.")
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Initialize ArUco dictionary and detector
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Detect ArUco markers in the grayscale image
        corners, ids, _ = aruco_detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

            # Estimate pose for each detected marker
            tag_size = 0.2  # meters
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, tag_size, self.camera_matrix, self.dist_coeffs
            )

            # Draw the detected markers on the original image
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            # We'll define the 3D coordinates for a 0.2m marker (adjust if needed).
            half_size = tag_size / 2.0
            object_points = np.array(
                [
                    [-half_size, half_size, 0.0],
                    [half_size, half_size, 0.0],
                    [half_size, -half_size, 0.0],
                    [-half_size, -half_size, 0.0],
                ],
                dtype=np.float32,
            )

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                corners_2d = corners[i][0].astype(np.float32)

                # Solve for pose with solvePnP
                ret, rvec, tvec = cv2.solvePnP(
                    object_points,
                    corners_2d,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if not ret:
                    self.get_logger().warning(f"SolvePnP failed for marker {marker_id}")
                    continue

                # Draw axes with cv2.drawFrameAxes
                axis_length = 0.1
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length, 2
                )

                pos_x, pos_y, pos_z = tvec.reshape(-1)
                # Publish to /aruco_marker_data
                marker_msg = Float32MultiArray()
                marker_msg.data = [float(marker_id), pos_x, pos_y, pos_z]
                self.aruco_pub.publish(marker_msg)

                self.get_logger().info(
                    f"Marker {marker_id}: Position -> x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}"
                )

        else:
            self.get_logger().debug("No ArUco markers detected in this frame.")

        # Scale and display for debugging
        scale_factor = 6.0
        resized_image = cv2.resize(
            cv_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR
        )
        cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("ArUco Detection", resized_image)
        cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   Lidar Processing
    # --------------------------------------------------------------------------
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

        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance:.2f}m")
        self.processed_pub.publish(Float32(data=min_distance))

    # --------------------------------------------------------------------------
    #   Depth-based Obstacle Detection
    # --------------------------------------------------------------------------
    def obstacleDetection(self, depth_msg: Image) -> None:
        """
        Uses the ZED stereo depth map to detect large, close obstacles.
        Publishes a boolean if an obstacle is detected, as well as obstacle details.

        Args:
            depth_msg (Image): ROS2 image message (32-bit float depth image).
        """
        if depth_msg.encoding not in ["32FC1", "16UC1"]:
            self.get_logger().warning(
                f"Unsupported depth image encoding: {depth_msg.encoding}. Expected 32FC1 or 16UC1."
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")
            return

        # Depth image is in meters (ZED typically publishes 32FC1).
        # We want to find if there's an object taller than ~0.3m (1 foot)
        # within ~3m range (10 feet ~ 3.05m). This is a simplistic approach.

        # We'll do a naive approach: Check some region(s) of interest
        # for depth values significantly less than 3m, forming a cluster
        # that might indicate a tall obstacle. A real solution might do
        # bounding box detection, morphological ops, or use the ZED SDK's
        # object detection API. Here, we do a basic threshold approach.

        # 1. Create a mask for "close" obstacles: depth < 3.0m
        close_thresh = 3.0
        # Because of 1-foot height requirement, we won't detect small bumps, but
        # in a raw depth image, we don't have direct object height. We only check
        # if something is close. (One could integrate the camera's angle or stereo
        # geometry to estimate height from multiple points in the cluster.)
        # For simplicity, let's assume if there are "enough" close pixels, it is
        # large enough to be an obstacle.

        close_mask = (depth_image > 0.0) & (depth_image < close_thresh)
        num_close_pixels = np.count_nonzero(close_mask)

        # 2. Heuristic: If more than X% of the image is under 3m, we might have a big obstacle
        height, width = depth_image.shape
        total_pixels = height * width
        threshold_percentage = 0.01  # 1% of image
        self_detected = False

        if total_pixels == 0:
            self.get_logger().warning("Depth image has zero size.")
            return

        if (num_close_pixels / total_pixels) > threshold_percentage:
            # We consider this an obstacle
            self_detected = True

        # Publish a bool for obstacle detection
        self.obstacle_detected_pub.publish(Bool(data=self_detected))

        # Additionally, we can publish some obstacle info
        # (e.g., fraction of close pixels, average distance of the "close" area)
        obstacle_data = Float32MultiArray()

        if self_detected:
            # Compute average distance of the close pixels
            avg_close_distance = float(np.mean(depth_image[close_mask]))
            obstacle_data.data = [
                1.0,  # 1 means we found an obstacle
                avg_close_distance,
                float(num_close_pixels),
            ]
            self.get_logger().info(
                f"Obstacle detected. Average distance ~ {avg_close_distance:.2f} m"
            )
        else:
            obstacle_data.data = [0.0, 0.0, 0.0]  # No obstacle

        self.obstacle_info_pub.publish(obstacle_data)


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
        cv2.destroyAllWindows()
        sensor_processing_node.destroy_node()
        rclpy.shutdown()
        # Not forcibly calling sys.exit(0). Let the normal shutdown proceed.


if __name__ == "__main__":
    main()
