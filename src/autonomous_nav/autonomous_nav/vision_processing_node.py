import struct
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
from cv2 import aruco
from cv_bridge import CvBridge
from octomap_msgs.msg import Octomap as OctomapMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Float32MultiArray, Header

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO
from visualization_msgs.msg import MarkerArray


class VisionProcessingNode(Node):
    """
    A ROS 2 node for processing camera data with:
      - ArUco marker detection
      - Custom object detection for "Hammer" + "Bottle" using YOLO World
    """

    def __init__(self) -> None:
        super().__init__("vision_processing_node")

        self.get_logger().info("Initializing vision_processing_node...")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        # Load YOLO World model
        self.model = YOLO("yolov8l-world.pt")
        self.get_logger().info("YOLO World model loaded successfully.")

        self.camera_frame_id = "zed_camera_frame"
        self.map_frame_id = "map"
        
        # Aruco detector setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ----------------------------------------------------------------------
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.combinedCallback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        # Publishers
        self.object_detection_pub = self.create_publisher(
            Image, "/object_detection_image", 10
        )

        self.get_logger().info("vision_processing_node is up and running.")


    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)


    # --------------------------------------------------------------------------
    #   Combined callback
    # --------------------------------------------------------------------------

    def combinedCallback(self, msg: Image) -> None:
        """
        Combined callback to run both YOLO World detection and ArUco marker detection.
        """

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        self.arucoMarkerDetection(frame)
        
        # This resizes the frame for yolo, if its not accurate enough maybe increase the size
        resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        self.yoloDetectionCallback(resized)
    # --------------------------------------------------------------------------
    #   YOLO World Object Detection
    # --------------------------------------------------------------------------
    def yoloDetectionCallback(self, msg: Image) -> None:
        """
        Runs YOLO World detection on the camera feed.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for YOLO World detection: {e}")
            return

        # Predict with YOLO World
        results = self.model(frame, conf=0.3)[0]  # Adjust confidence threshold if needed

        # Get frame dimensions
        frame_h, frame_w = frame.shape[:2]

        # Filter results for "hammer" and "bottle"
        for det in results.boxes.data:
            x1, y1, x2, y2, confidence, class_idx = det.tolist()
            label = self.model.names[int(class_idx)]  # Get detected object name
            confidence = float(confidence)

            if confidence < 0.3:  # Adjust confidence threshold if needed
                 continue

            # Pick color
            color = (255, 255, 255) if label.lower() == "bottle" else (0, 165, 255)

            # Draw bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            text = f"{label} {confidence:.2f}"
            cv2.putText(
                frame, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
            )

        # Display result
        #disp = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
        #cv2.imshow("YOLO World Detection", disp)
        #cv2.waitKey(1)

        #publish results to view with rviz
        disp = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
        image_message = self.bridge.cv2_to_imgmsg(disp, "passthrough")
        self.object_detection_pub.publish(image_message)

            
    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, frame: np.ndarray) -> None:
        """
        Basic ArUco marker detection with pose estimation
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)


        corners, ids, _ = self.aruco_detector.detectMarkers(gray_image)

        if ids is None or len(ids) == 0:
            return

        self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

        marker_length = 0.20 # Marker size in meters
        half_size = marker_length / 2.0

        obj_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0],
        ], dtype=np.float32)


        for i, marker_id in enumerate(ids.flatten()):
            img_points = corners[i][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                img_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not success:
                self.get_logger().warning(f"Pose estimation failed for marker ID {int(marker_id)}")
                continue

            tvec = tvec.reshape(3)
            rvec  = rvec.reshape(3)
                
            distance = float(np.linalg.norm(tvec))

            self.get_logger().info(
                f"Marker ID: {int(marker_id)}, Distance: {distance:.2f} m, tvec: {tvec}, rvec: {rvec}"
            )

            cv2.drawFrameAxes(
                cv_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                marker_length /2.0,
            )
    # --------------------------------------------------------------------------
    #   ROS 2 Node Main
    # --------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    vision_processing_node = None
    try:
        vision_processing_node = VisionProcessingNode()
        rclpy.spin(vision_processing_node)
    except KeyboardInterrupt:
        if vision_processing_node is not None:
            vision_processing_node.get_logger().info(
                "Keyboard interrupt received. Shutting down..."
            )
    finally:
        if vision_processing_node is not None:
            cv2.destroyAllWindows()
            vision_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
