"""This node computes where the Pose is for the current character that the arm should move to"""

from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import String

from src.lib.vision.aruco_detection import ArucoFrame


class KeyboardPoseNode(Node):
    """
    Takes a typing message (single character) and publishes a global Pose for the arm to move to
    """

    def __init__(self) -> None:
        self.message: String = ""
        self.frame: Optional[ArucoFrame] = None

        # ============ Subscribers ============
        self.message_subscriber = self.create_subscription(
            String, "arm/typing/message", self.message_callback, 1
        )

        self.frame_subscriber = self.create_subscription(
            ArucoFrame, "arm/typing/message", self.frame_callback, 10
        )

        # ============ Publishers ============
        self.arm_pose_pub = self.create_publisher(Pose, "arm/pose", 1)

    # ============ Callbacks ============
    def message_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received character to type {msg}")
        self.message = msg.data

    def frame_callback(self, msg: ArucoFrame) -> None:
        self.get_logger().info("Received Aruco Frame to work within")
        self.frame = msg

    # ============ Main Logic ============
    def publish_arm_pose(self) -> None:
        if self.frame is None or self.message == "":
            return

    def compute_pose_for_character(self, char: str) -> Pose:
        # TODO Use key_positions.json to map where characters are
        # TODO Map Pose within ArucoFrame to global Pose for arm to move to
        pose = Pose()
        return pose
