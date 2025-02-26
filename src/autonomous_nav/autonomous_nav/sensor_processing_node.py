"""
Role: This script defines the sensor processing node, responsible for processing all sensor data, 
primarily camera images, for object and obstacle detection. Data is published in a format suitable 
for other nodes.

Functionality:
- Subscribe to images and process for obstacle & object recognition with OpenCV.
- Outputs data to the Decision Making and GPS Anchor Nodes as needed.
- Detects large, close obstacles via the ZED stereo depth map.
- Publishes obstacle data for the DecisionMakingNode to use.

Adjustments to reduce false positives:
- Decreases detection range to ~1.5m (5 feet).
- Applies morphological filtering on the depth mask to ignore small clusters (e.g., tall grass).
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
        # Subscribe to the ZED node's depth image
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
        """
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Processes the incoming camera image to detect ArUco markers.
        Draws markers and axes onto the image and displays the result (for debugging).
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

        # Debug display (upscale)
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
        Publishes:
         - /obstacle_detected (Bool)
         - /obstacle_info (Float32MultiArray)
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

        # ---------------------------------------------------------
        #    1) Threshold: Detect "close" obstacles within ~1.5m
        # ---------------------------------------------------------
        close_thresh = 1.5  # ~5 feet
        mask_close = (depth_image > 0.0) & (depth_image < close_thresh)

        # ---------------------------------------------------------
        #    2) Morphological filtering to remove small patches
        #       (e.g., grass, small rocks).
        # ---------------------------------------------------------
        # Convert to 8-bit mask for morphological ops
        mask_uint8 = np.where(mask_close, 255, 0).astype(np.uint8)

        # Use a small kernel to remove noise
        kernel = np.ones((5, 5), np.uint8)
        # 'Open' = erode then dilate => remove small bright regions (noise)
        # 'Close' = dilate then erode => fill small holes.
        # We'll do an 'open' to remove small patches, then a 'close' to fuse adjacent patches.
        opened = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
        filtered_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        # Count how many "close" pixels remain after filtering
        num_close_pixels = np.count_nonzero(filtered_mask)

        # ---------------------------------------------------------
        #    3) Decide if it's a 'large' obstacle
        # ---------------------------------------------------------
        height, width = depth_image.shape
        total_pixels = height * width
        if total_pixels == 0:
            self.get_logger().warning("Depth image has zero size.")
            return

        # If more than 1% of the image is in the filtered mask, consider it an obstacle
        threshold_percentage = 0.01
        self_detected = False

        if (num_close_pixels / total_pixels) > threshold_percentage:
            self_detected = True

        # Publish detection as a Bool
        self.obstacle_detected_pub.publish(Bool(data=self_detected))

        # ---------------------------------------------------------
        #    4) Publish additional info
        # ---------------------------------------------------------
        obstacle_data = Float32MultiArray()
        if self_detected:
            # Compute average distance over 'filtered_mask' area
            close_pixels = depth_image[filtered_mask == 255]
            avg_close_distance = float(np.mean(close_pixels)) if close_pixels.size > 0 else 0.0

            obstacle_data.data = [
                1.0,  # 1 means we found an obstacle
                avg_close_distance,
                float(num_close_pixels),
            ]
            self.get_logger().info(
                f"Obstacle detected within {close_thresh:.1f}m. Avg distance ~ {avg_close_distance:.2f} m"
            )
        else:
            obstacle_data.data = [0.0, 0.0, 0.0]

        self.obstacle_info_pub.publish(obstacle_data)


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the SensorProcessingNode.
    """
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
