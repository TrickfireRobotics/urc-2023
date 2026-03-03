from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from src.lib.vision.aruco_detection import ArucoFrame

from .spacebar_detector import SpacebarDetector


class KeyboardPoseNode(Node):
    """
    Takes a typing message (single character) and publishes a global Pose for the arm to move to
    """

    def __init__(self) -> None:
        super().__init__("keyboard_pose_node")
        self.message: String = ""
        self.aruco_frame: Optional[ArucoFrame] = None
        self.frame: Optional[np.ndarray] = None
        self.distance_map: dict[str, tuple[float, float]] = (
            {}
        )  # maps characters to (x, y) positions on keyboard frame

        # ============ Subscribers ============
        self.message_subscriber = self.create_subscription(
            String, "arm/typing/message", self.message_callback, 1
        )

        self.frame_subscriber = self.create_subscription(
            ArucoFrame, "arm/keyboard_frame", self.frame_callback, 10
        )

        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.camera_frame_callback, 10
        )

        # ============ Publishers ============
        self.arm_pose_pub = self.create_publisher(Pose, "arm/pose", 1)

    # ============ Callbacks ============
    def message_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received character to type {msg}")
        self.message = msg.data

    def frame_callback(self, msg: ArucoFrame) -> None:
        self.get_logger().info("Received Aruco Frame to work within")
        self.aruco_frame = msg

    def camera_frame_callback(self, msg: Image) -> None:
        """Isolates each image for the aruco detector"""
        self.get_logger().info("Camera frame received")
        self.frame = np.ndarray(msg.data, np.uint8)

    def publish_arm_pose(self) -> None:
        if self.frame is None or self.message == "":
            pose = self.compute_pose_for_character(self.message)
            self.arm_pose_pub.publish(pose)

    # ============ Main Logic ============
    def load_distance_map(
        self, path: str = "src/arm/arm/autonomous_arm/key_positions.json"
    ) -> None:
        """Loads the distance map from a JSON file that maps characters to (x, y) positions on the keyboard frame."""
        import json

        with open(path, "r") as f:
            self.distance_map = json.load(f)

    def compute_pose_for_character(self, char: str) -> Optional[Pose]:
        # TODO Use key_positions.json to map where characters are
        # TODO Map Pose within ArucoFrame to global Pose for arm to move to
        if self.frame is None:
            self.get_logger().info("No Aruco frame available for pose computation")
            return None

        self.load_distance_map()
        if self.distance_map == {}:
            self.get_logger().info("Distance map is empty, cannot compute character positions")
            return None

        spacebar_detector = SpacebarDetector()
        x, y = spacebar_detector.detect_spacebar_center(
            frame=self.frame, aruco_frame=self.aruco_frame
        )
        self.get_logger().info(f"Detected spacebar center at ({x}, {y}) in keyboard frame")

        # location of character on keyboard frame can be computed using key_positions.json and the detected spacebar center
        char_coords = (
            self.distance_map.get(self.message, (0, 0))[0] + x,
            self.distance_map.get(self.message, (0, 0))[1] + y,
        )

        # Use tf to transform from keyboard frame to global frame for the arm to move to

        return Pose()
