"""
Role: This script defines the sensor processing node, responsible for processing all sensor data, 
including object detection in the ZED stereo depth map, while ignoring the rover's wheel and ground.

Functionality:
- Applies morphological filtering and bounding regions to exclude outer 25% (wheel) and bottom 25% 
  (ground).
- Publishes an obstacle detection boolean and additional info for the DecisionMakingNode.
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

    This node:
        - Subscribes to camera images for ArUco detection.
        - Subscribes to a depth image from the ZED stereo camera for obstacle detection.
        - Excludes outer 25% horizontally (wheels) and bottom 25% vertically (ground).
        - Uses morphological filtering to reduce false positives from grass or small objects.
        - Publishes a bool (/obstacle_detected) and additional info (/obstacle_info).
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")
        self.get_logger().info("Initializing sensor_processing_node...")

        # OpenCV bridge for converting ROS images
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # Subscribe to color image + camera info
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # Publisher for ArUco marker data
        self.aruco_pub = self.create_publisher(Float32MultiArray, "/aruco_marker_data", 10)

        # Subscribe to LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

        # Subscribe to ZED depth image
        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.obstacleDetection, 10
        )

        # Obstacle detection publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.obstacle_info_pub = self.create_publisher(Float32MultiArray, "/obstacle_info", 10)

        self.get_logger().info("sensor_processing_node is up and running, waiting for data...")

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco detection. (Unchanged from previous)
        """
        self.get_logger().debug("arucoMarkerDetection callback triggered.")
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = aruco_detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")
            tag_size = 0.2  # meters
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, tag_size, self.camera_matrix, self.dist_coeffs
            )

            # Draw markers
            aruco.drawDetectedMarkers(cv_image, corners, ids)
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

                axis_length = 0.1
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length, 2
                )

                pos_x, pos_y, pos_z = tvec.reshape(-1)
                marker_msg = Float32MultiArray()
                marker_msg.data = [float(marker_id), pos_x, pos_y, pos_z]
                self.aruco_pub.publish(marker_msg)

                self.get_logger().info(
                    f"Marker {marker_id}: Position -> x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}"
                )
        else:
            self.get_logger().debug("No ArUco markers detected in this frame.")

        # Debug display
        scale_factor = 6.0
        resized_image = cv2.resize(
            cv_image, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR
        )
        cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("ArUco Detection", resized_image)
        cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   processLidar
    # --------------------------------------------------------------------------
    def processLidar(self, msg: LaserScan) -> None:
        """
        Publishes min lidar distance as an example.
        """
        self.get_logger().debug("processLidar callback triggered.")
        if not msg.ranges:
            self.get_logger().warning("Lidar ranges data is empty.")
            return

        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance:.2f}m")
        self.processed_pub.publish(Float32(data=min_distance))

    # --------------------------------------------------------------------------
    #   obstacleDetection
    # --------------------------------------------------------------------------
    def obstacleDetection(self, depth_msg: Image) -> None:
        """
        Uses the ZED stereo depth map to detect large, close obstacles.
        Excludes the outer 25% of the width (wheel region) and bottom 25% (ground).
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

        height, width = depth_image.shape
        if height == 0 or width == 0:
            self.get_logger().warning("Depth image has zero size.")
            return

        # 1) Define region of interest:
        #    - Skip bottom 25% to avoid ground
        #    - Skip left/right 25% to avoid wheels
        roi_top = 0
        roi_bottom = int(0.75 * height)
        roi_left = int(0.25 * width)
        roi_right = int(0.75 * width)

        roi = depth_image[roi_top:roi_bottom, roi_left:roi_right]
        roi_h, roi_w = roi.shape

        # 2) Threshold for "close" obstacles within ~1.5m
        close_thresh = 1.5  # about 5 feet
        mask_close = (roi > 0.0) & (roi < close_thresh)

        # 3) Morphological filtering
        mask_uint8 = np.where(mask_close, 255, 0).astype(np.uint8)
        # Use a 9x9 kernel for strong noise removal
        kernel = np.ones((9, 9), np.uint8)
        opened = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
        filtered_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        num_close_pixels = np.count_nonzero(filtered_mask)
        total_pixels_roi = roi_h * roi_w

        # 4) Check if more than 1% of ROI is "close"
        threshold_percentage = 0.01
        self_detected = False
        if total_pixels_roi > 0 and (num_close_pixels / total_pixels_roi) > threshold_percentage:
            self_detected = True

        # Publish detection result
        self.obstacle_detected_pub.publish(Bool(data=self_detected))

        # 5) Publish obstacle info
        obstacle_data = Float32MultiArray()
        if self_detected:
            close_pixels = roi[filtered_mask == 255]
            avg_close_distance = float(np.mean(close_pixels)) if close_pixels.size > 0 else 0.0

            obstacle_data.data = [1.0, avg_close_distance, float(num_close_pixels)]
            self.get_logger().info(
                f"Obstacle in ROI. ~{avg_close_distance:.2f}m average distance. ROI used: {roi_w}x{roi_h}"
            )
        else:
            obstacle_data.data = [0.0, 0.0, 0.0]

        self.obstacle_info_pub.publish(obstacle_data)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    sensor_processing_node = None
    try:
        sensor_processing_node = SensorProcessingNode()
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                "Keyboard interrupt received. Shutting down..."
            )
    except ExternalShutdownException:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                colorStr(
                    "External shutdown requested. Shutting down sensor_processing_node",
                    ColorCodes.BLUE_OK,
                )
            )
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
