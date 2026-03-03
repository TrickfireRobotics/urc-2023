from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from src.lib.vision.aruco_detection import ArucoDetection, ArucoFrame, ArucoMarkerResult


class KeyboardLocatorNode(Node):
    """
    This node takes in camera intrinsics and camera_id to compute bounding box around keyboard

    TODO Update camera info subscribers to use camera that will be on arm - currently using ZED cam
    TODO Check what aruco dict to use for typing mission - currently using 6x6_250
    """

    def __init__(self) -> None:
        super().__init__("keyboard_locator_node")
        self.target_char: str = ""
        self.camera_frame_id: Optional[int] = None
        self.frame: Optional[np.ndarray] = None

        # ============ Subscribers ============
        self.camera_frame_subscriber = self.create_subscription(
            int, "keyboard_camera_frame_id", self.camera_frame_id_callback, 1
        )

        # MUST BE REPLACED WITH ARM CAMERA TOPICS
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.process_camera_info, 10
        )

        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.camera_frame_callback, 10
        )

        # ============ Publishers ============
        self.keyboard_frame_pub = self.create_publisher(ArucoFrame, "arm/keyboard_frame", 10)

        self.create_timer(0.2, self.publish_keyboard_frame)

    def keyboard_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received input character: {msg.data}")
        self.target_char = msg.data

    def camera_frame_id_callback(self, msg: int) -> None:
        self.get_logger().info(f"Received keyboard camera frame id: {msg.data}")
        self.camera_frame_id = msg.data

    def process_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def camera_frame_callback(self, msg: Image) -> None:
        """Isolates each image for the aruco detector"""
        # im: np.ndarray[np.uint8] = np.ndarray(msg.data, np.uint8)
        # self.frame = im
        self.get_logger().info("Camera frame received")
        self.frame = np.ndarray(msg.data, np.uint8)

    # ============ Main Logic ============

    def publish_keyboard_frame(self) -> None:
        frame: Optional[ArucoFrame] = self.compute_frame()
        if frame is not None:
            self.keyboard_frame_pub.publish(frame)
        self.get_logger().info("Camera frame is None")

    def compute_frame(self) -> Optional[ArucoFrame]:
        """Uses the aruco detection class and key"""
        assert self.camera_frame_id is not None and self.frame is not None
        detector: ArucoDetection = ArucoDetection(
            dict_type=cv2.aurco.DICT_6x6_250,
            marker_length=1.0,
            cam_id=self.camera_frame_id,
        )
        detector.load_calibration(self.camera_matrix, self.dist_coeffs)

        markers: list[ArucoMarkerResult] = detector.detect_markers(self.frame)
        if markers is None:
            self.get_logger().info(f"No markers detected in frame {self.camera_frame_id}")

        # Compute frame that the keyboard is in (surrounded by aruco markers)
        return detector.compute_frame(markers)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    keyboard_locator_node = None
    try:
        keyboard_locator_node = KeyboardLocatorNode()
        rclpy.spin(keyboard_locator_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if keyboard_locator_node is not None:
            keyboard_locator_node.get_logger().info("Shutting down keyboard_locator_node")
    finally:
        if keyboard_locator_node is not None:
            keyboard_locator_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
