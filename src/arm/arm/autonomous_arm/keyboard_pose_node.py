import json
from typing import Optional

import cv2
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from src.lib.vision.aruco_detection import ArucoDetection, ArucoMarkerResult

from .spacebar_detector import SpacebarDetector


class KeyboardPoseNode(Node):
    """
    Receives a character to type (on arm/typing/next_char), detects the keyboard via
    ArUco markers, and publishes a global Pose in base_link for the arm to move to.
    """

    def __init__(self) -> None:
        super().__init__("keyboard_pose_node")
        self.message: str = ""
        self.frame: Optional[np.ndarray] = None
        self.distance_map: dict[str, list[float]] = {}
        self.detector: Optional[ArucoDetection] = None

        # Physical keyboard dimensions — tunable without recompile
        self.declare_parameter("keyboard_width_m", 0.360)  # ~14.2 inches, standard TKL
        self.declare_parameter("keyboard_height_m", 0.150)  # ~5.9 inches
        self.declare_parameter("marker_length_m", 0.0529)  # physical ArUco marker side

        # TF2 — buffer stores the transform tree; listener feeds it from /tf and /tf_static
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ============ Subscribers ============
        # Receives individual characters to type from typing_coordinator_node
        self.message_subscriber = self.create_subscription(
            String, "arm/typing/next_char", self.message_callback, 1
        )
        # MUST BE REPLACED WITH ARM CAMERA TOPICS
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.process_camera_info, 10
        )
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.camera_frame_callback, 10
        )

        # ============ Publishers ============
        self.arm_pose_pub = self.create_publisher(Pose, "arm/pose", 1)

        self.load_distance_map()

    # ============ Callbacks ============

    def message_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received character to type: {msg.data!r}")
        self.message = msg.data
        self.publish_arm_pose()

    def process_camera_info(self, msg: CameraInfo) -> None:
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(msg.d, dtype=np.float64)
        marker_length = self.get_parameter("marker_length_m").get_parameter_value().double_value
        self.detector = ArucoDetection(
            dict_type=cv2.aruco.DICT_6X6_250,
            marker_length=marker_length,
            cam_id=0,
        )
        self.detector.load_calibration(camera_matrix, dist_coeffs)

    def camera_frame_callback(self, msg: Image) -> None:
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    # ============ Main Logic ============

    def load_distance_map(
        self, path: str = "src/arm/arm/autonomous_arm/key_positions.json"
    ) -> None:
        """Load key offsets (metres from spacebar centre) from JSON."""
        try:
            with open(path, "r") as f:
                self.distance_map = json.load(f)
            self.get_logger().info(f"Loaded distance map with {len(self.distance_map)} keys")
        except FileNotFoundError:
            self.get_logger().error(f"Distance map not found at {path!r}")

    def publish_arm_pose(self) -> None:
        if self.frame is None or self.message == "":
            self.get_logger().warn("Frame or message not ready — cannot publish arm pose")
            return
        pose = self.compute_pose_for_character(self.message)
        if pose is not None:
            self.arm_pose_pub.publish(pose)
            self.get_logger().info(f"Published arm pose for '{self.message}'")

    def compute_pose_for_character(self, char: str) -> Optional[Pose]:
        """
        Compute a Pose in base_link frame for the arm to press key `char`.

        Steps:
          1. Run ArUco detection on current camera frame to get the keyboard bounding box.
          2. Run SpacebarDetector to get spacebar center in pixel coords.
          3. Convert pixel coords → metres using physical keyboard dimensions.
          4. Add key_positions.json offset (metres, relative to spacebar) → key position in keyboard_frame.
          5. Express as a PointStamped in keyboard_frame.
          6. Use TF2 to transform to base_link.
          7. Populate and return Pose.
        """
        char_upper = char.upper()

        if self.frame is None:
            self.get_logger().warn("No camera frame available")
            return None
        if self.detector is None:
            self.get_logger().warn("Camera not calibrated yet")
            return None
        if not self.distance_map:
            self.get_logger().warn("Distance map not loaded")
            return None
        if char_upper not in self.distance_map:
            self.get_logger().warn(f"Character '{char}' not in distance map")
            return None

        # Step 1: Detect ArUco markers + compute 2D bounding box
        markers: list[ArucoMarkerResult] = self.detector.detect_markers(self.frame)
        aruco_frame = self.detector.compute_frame(markers)
        if aruco_frame is None:
            self.get_logger().warn("Need >=2 ArUco markers to compute keyboard frame")
            return None

        # Step 2: Find spacebar center in pixel space
        sb_result = SpacebarDetector().detect_spacebar(frame=self.frame, aruco_frame=aruco_frame)
        if sb_result is None:
            self.get_logger().warn("Spacebar not detected in current frame")
            return None
        sb_x_px, sb_y_px = sb_result

        # Step 3: Pixel → metre conversion
        kw = self.get_parameter("keyboard_width_m").get_parameter_value().double_value
        kh = self.get_parameter("keyboard_height_m").get_parameter_value().double_value
        x_scale = kw / aruco_frame.width  # metres per pixel (X)
        y_scale = kh / aruco_frame.height  # metres per pixel (Y)

        # Spacebar position relative to keyboard_frame origin (= aruco bounding box centre)
        sb_x_m = (sb_x_px - aruco_frame.center[0]) * x_scale
        sb_y_m = (sb_y_px - aruco_frame.center[1]) * y_scale

        # Step 4: Add key_positions.json offset (already in metres, relative to spacebar)
        offset = self.distance_map[char_upper]
        key_x_m = sb_x_m + offset[0]
        key_y_m = sb_y_m + offset[1]

        # Step 5: Build PointStamped in keyboard_frame
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "keyboard_frame"
        pt.point.x = key_x_m
        pt.point.y = key_y_m
        pt.point.z = 0.0  # on the keyboard surface

        # Step 6: TF lookup keyboard_frame → base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "keyboard_frame",
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF lookup failed (no transform yet?): {e}")
            return None
        except tf2_ros.ConnectivityException as e:
            self.get_logger().error(f"TF lookup failed (connectivity): {e}")
            return None
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"TF lookup failed (extrapolation): {e}")
            return None

        pt_in_base = tf2_geometry_msgs.do_transform_point(pt, transform)

        # Step 7: Populate Pose
        # Position: key location in base_link frame
        # Orientation: keyboard_frame's orientation in base_link → arm approaches perpendicular to keyboard
        pose = Pose()
        pose.position.x = pt_in_base.point.x
        pose.position.y = pt_in_base.point.y
        pose.position.z = pt_in_base.point.z
        pose.orientation = transform.transform.rotation
        return pose


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    keyboard_pose_node = None
    try:
        keyboard_pose_node = KeyboardPoseNode()
        rclpy.spin(keyboard_pose_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if keyboard_pose_node is not None:
            keyboard_pose_node.get_logger().info("Shutting down keyboard_pose_node")
    finally:
        if keyboard_pose_node is not None:
            keyboard_pose_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
