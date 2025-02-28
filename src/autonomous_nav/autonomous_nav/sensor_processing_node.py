"""
Role: This script defines the sensor processing node, responsible for processing all sensor data,
including:
  - ArUco detection (unchanged)
  - LIDAR min-distance reading (unchanged)
  - Depth-based obstacle detection ignoring wheels/ground (unchanged)
  - New YOLO-based detection for approximate "water bottle" (COCO label "bottle") 
    and "orange mallet" (COCO label "sports ball" or "baseball bat").

Uses a pretrained YOLO model from ultralytics (yolov8n.pt) for a quick demonstration.
"""

import sys
import math
import numpy as np
import cv2  # pylint: disable=no-member
from cv_bridge import CvBridge
from cv2 import aruco
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ROS messages
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray, Bool

# YOLO from ultralytics (pip install ultralytics)
from ultralytics import YOLO

from lib.color_codes import ColorCodes, colorStr


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data.

    Existing functionality:
      - ArUco detection on /zed/zed_node/rgb/image_rect_color
      - Depth-based obstacle detection on /zed/zed_node/depth/depth_registered
        ignoring wheels & ground
      - LIDAR min-distance
    New:
      - YOLO-based detection for classes:
          "bottle" -> water bottle
          "sports ball" (or "baseball bat") -> orange mallet

    We draw bounding boxes on the camera feed with YOLO in a new window "YOLO Object Detection."
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")
        self.get_logger().info("Initializing sensor_processing_node with YOLO demo...")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # --------------------------------------------------
        #  Load YOLO model (pretrained, e.g. 'yolov8n.pt')
        # --------------------------------------------------
        # The default YOLOv8n model is small and covers 80 COCO classes.
        # We'll filter for "bottle" & "sports ball" or "baseball bat."
        self.model = YOLO("yolov8n.pt")  # download auto if not present

        # --------------------------------------------------
        #  Subscriptions
        # --------------------------------------------------
        # 1) ArUco detection
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # 2) YOLO-based detection
        self.yolo_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.yoloObjectDetectionCallback, 10
        )

        # 3) LIDAR
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

        # 4) Depth-based obstacle detection
        self.depth_sub = self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.obstacleDetection, 10
        )
        self.obstacle_detected_pub = self.create_publisher(Bool, "/obstacle_detected", 10)
        self.obstacle_info_pub = self.create_publisher(Float32MultiArray, "/obstacle_info", 10)

        # ArUco publisher
        self.aruco_pub = self.create_publisher(Float32MultiArray, "/aruco_marker_data", 10)

        self.get_logger().info("sensor_processing_node is up and running.")

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   yoloObjectDetectionCallback
    # --------------------------------------------------------------------------
    def yoloObjectDetectionCallback(self, msg: Image) -> None:
        """
        Runs YOLO inference on each frame to detect approximate classes for
        a water bottle -> "bottle"
        an orange mallet -> e.g., "sports ball" or "baseball bat" in COCO
        Then draws bounding boxes in a window "YOLO Object Detection."
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for YOLO detection: {e}")
            return

        # YOLO inference with default confidence threshold
        results = self.model.predict(source=frame, conf=0.2)

        if len(results) > 0:
            # YOLOv8 returns a list of results; we take the first (single image)
            detections = results[0].boxes  # bounding boxes
            class_names = results[0].names  # {0: 'person', 1: 'bicycle', 39: 'bottle', etc.}

            for det in detections:
                cls_id = int(det.cls[0].item())  # class index
                label = class_names[cls_id] if cls_id in class_names else f"cls{cls_id}"
                conf_score = float(det.conf[0].item())
                print(f"Detected class = {label}, conf = {conf_score}")

                # We'll watch for "bottle" or "sports ball" or "baseball bat"
                # The default COCO classes are: "bottle", "sports ball", "baseball bat"
                # that might approximate your orange mallet
                if label.lower() in ["bottle", "sports ball", "baseball bat"]:
                    # bounding box coords
                    x1, y1, x2, y2 = det.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # pick color by label (optional)
                    if label.lower() == "bottle":
                        color = (255, 255, 255)
                    else:
                        color = (0, 165, 255)  # orange-ish

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    text = f"{label} {conf_score:.2f}"
                    cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Display YOLO results
        scale_factor = 2.0
        disp = cv2.resize(
            frame, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR
        )
        cv2.namedWindow("YOLO Object Detection", cv2.WINDOW_NORMAL)
        cv2.imshow("YOLO Object Detection", disp)
        cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco detection (unchanged).
        """
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
            # If you do pose estimation, etc. keep your existing logic here.

        # (Optional: display the ArUco detection result in another window)

    # --------------------------------------------------------------------------
    #   processLidar
    # --------------------------------------------------------------------------
    def processLidar(self, msg: LaserScan) -> None:
        self.get_logger().debug("processLidar callback triggered.")
        if not msg.ranges:
            self.get_logger().warning("Lidar ranges data is empty.")
            return

        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance:.2f}m")
        self.processed_pub.publish(Float32(data=min_distance))

    # --------------------------------------------------------------------------
    #   obstacleDetection (Depth)
    # --------------------------------------------------------------------------
    def obstacleDetection(self, depth_msg: Image) -> None:
        """
        Depth-based obstacle detection ignoring outer 25% (wheels) + bottom 25% (ground).
        (Unchanged from original)
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

        # define region of interest
        roi_top = 0
        roi_bottom = int(0.75 * height)
        roi_left = int(0.25 * width)
        roi_right = int(0.75 * width)
        roi = depth_image[roi_top:roi_bottom, roi_left:roi_right]
        roi_h, roi_w = roi.shape

        close_thresh = 1.5
        mask_close = (roi > 0.0) & (roi < close_thresh)

        mask_uint8 = np.where(mask_close, 255, 0).astype(np.uint8)
        kernel = np.ones((9, 9), np.uint8)
        opened = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, kernel)
        filtered_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        num_close_pixels = np.count_nonzero(filtered_mask)
        total_pixels_roi = roi_h * roi_w
        threshold_percentage = 0.01
        self_detected = False

        if total_pixels_roi > 0 and (num_close_pixels / total_pixels_roi) > threshold_percentage:
            self_detected = True

        self.obstacle_detected_pub.publish(Bool(data=self_detected))

        obstacle_data = Float32MultiArray()
        if self_detected:
            close_pixels = roi[filtered_mask == 255]
            avg_close_distance = float(np.mean(close_pixels)) if close_pixels.size > 0 else 0.0
            obstacle_data.data = [1.0, avg_close_distance, float(num_close_pixels)]
            self.get_logger().info(
                f"Obstacle in ROI. ~{avg_close_distance:.2f}m average distance. ROI: {roi_w}x{roi_h}"
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
