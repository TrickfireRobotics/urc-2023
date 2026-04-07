from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from src.lib.vision.aruco_detection import ArucoDetection, ArucoMarkerResult


class KeyboardLocatorNode(Node):
    """
    Detects ArUco markers surrounding the keyboard in the arm camera feed and
    broadcasts the keyboard's 3D pose as a TF frame (arm_camera_link → keyboard_frame).

    TODO: Update camera topics to use the arm-mounted camera once available.
    """

    def __init__(self) -> None:
        super().__init__("keyboard_locator_node")
        self.frame: Optional[np.ndarray] = None
        self.detector: Optional[ArucoDetection] = None

        # TF broadcaster — publishes arm_camera_link → keyboard_frame dynamically
        self.tf_broadcaster = TransformBroadcaster(self)

        # ============ Subscribers ============
        # MUST BE REPLACED WITH ARM CAMERA TOPICS
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.process_camera_info, 10
        )
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.camera_frame_callback, 10
        )

        # Detect markers and broadcast keyboard_frame at 5 Hz
        self.create_timer(0.2, self.detect_and_broadcast)

    def process_camera_info(self, msg: CameraInfo) -> None:
        camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(msg.d, dtype=np.float64)
        self.detector = ArucoDetection(
            dict_type=cv2.aruco.DICT_6X6_250,
            marker_length=0.0529,  # physical ArUco marker side length in metres
            cam_id=0,
        )
        self.detector.load_calibration(camera_matrix, dist_coeffs)

    def camera_frame_callback(self, msg: Image) -> None:
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    # ============ Main Logic ============

    def detect_and_broadcast(self) -> None:
        """Detect ArUco markers and broadcast keyboard_frame to TF."""
        if self.frame is None or self.detector is None:
            return

        markers: list[ArucoMarkerResult] = self.detector.detect_markers(self.frame)

        tvecs = [m.tvec.flatten() for m in markers if m.tvec is not None]
        if not tvecs:
            self.get_logger().debug("No markers with pose data detected")
            return

        # Keyboard frame origin = centroid of all marker positions in camera frame
        t_mean = np.mean(tvecs, axis=0)

        # Keyboard frame orientation = first detected marker's rotation
        first = next((m for m in markers if m.rvec is not None), None)
        if first is None:
            return
        R, _ = ArucoDetection.get_marker_transform(first.rvec, first.tvec)
        quat = self._rotation_matrix_to_quaternion(R)  # (x, y, z, w)

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "arm_camera_link"
        ts.child_frame_id = "keyboard_frame"
        ts.transform.translation.x = float(t_mean[0])
        ts.transform.translation.y = float(t_mean[1])
        ts.transform.translation.z = float(t_mean[2])
        ts.transform.rotation.x = float(quat[0])
        ts.transform.rotation.y = float(quat[1])
        ts.transform.rotation.z = float(quat[2])
        ts.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(ts)
        self.get_logger().debug(
            f"Broadcast keyboard_frame: t={t_mean.round(3)}, n_markers={len(tvecs)}"
        )

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
        """Convert a 3x3 rotation matrix to (x, y, z, w) quaternion (Shepperd method)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)


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
