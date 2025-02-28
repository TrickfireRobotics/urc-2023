import sys
import math
import numpy as np
import cv2  # pylint: disable=no-member
from cv2 import aruco
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data with:
      - ArUco marker detection
      - Depth-based obstacle ignoring wheels/ground
      - LIDAR min-distance
      - Pretrained YOLOv8 object detection for "Hammer" + "Bottle"
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        self.get_logger().info("Initializing sensor_processing_node with YOLOv8 detection...")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # ----------------------------------------------------------------------
        # Load a valid YOLOv8 detection model trained on COCO dataset
        self.model = YOLO("yolov8m.pt")  # Use "yolov8m.pt" or "yolov8s.pt" for faster inference

        # ----------------------------------------------------------------------
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # Object Detection with YOLOv8
        self.yolo_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.yoloDetectionCallback, 10
        )

        self.get_logger().info("sensor_processing_node is up and running with YOLOv8.")

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   YOLOv8 Object Detection
    # --------------------------------------------------------------------------
    def yoloDetectionCallback(self, msg: Image) -> None:
        """
        Runs a YOLOv8 detection model on the camera feed.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for YOLOv8 detection: {e}")
            return

        # Predict with YOLO
        results = self.model(frame, conf=0.1)[0]  # Run detection on the frame

        # Get frame dimensions
        frame_h, frame_w = frame.shape[:2]

        # Filter results for "hammer" and "bottle"
        for det in results.boxes.data:
            x1, y1, x2, y2, confidence, class_idx = det.tolist()
            label = self.model.names[int(class_idx)]  # Get class name from YOLO model
            confidence = float(confidence)

            if label.lower() in ["hammer", "bottle"]:
                # Pick color
                color = (255, 255, 255) if label.lower() == "bottle" else (0, 165, 255)

                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                text = f"{label} {confidence:.2f}"
                cv2.putText(
                    frame, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
                )

        # Display result
        disp = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
        cv2.imshow("YOLOv8 Detection", disp)
        cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco marker detection.
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

    # --------------------------------------------------------------------------
    #   ROS 2 Node Main
    # --------------------------------------------------------------------------


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
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
