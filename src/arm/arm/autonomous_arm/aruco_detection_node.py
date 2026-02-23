# This node takes in Aruco points relative to the camera, transforms them to the robots base frame, and publishes them for use in the arm control node
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from lib.vision.aruco_detection import ArucoDetection


# TODO: get the camera matrix and distortion coefficients from a subscription instead of hardcoding them
# TODO: publish the Aruco points in the robots base frame to a topic
class aruco_detection_node(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detection_node")
        self.detector = ArucoDetection(
            dict_type=cv2.aruco.DICT_6X6_250,
            marker_length=0.0529,
        )
        # get the camera_matrix, dist_coeffs from the camera calibration
        camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))
        self.detector.load_calibration(camera_matrix, dist_coeffs)
        self.tf_broadcaster = TransformBroadcaster(self)

    # This function sets up the camera connection and returns the connection object
    def setup_camera(self) -> Optional[cv2.VideoCapture]:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            # logging.error("Failed to open camera")
            return None
        return cap

    def publish_marker_tf(self, rvec: np.ndarray, tvec: np.ndarray, marker_id: int) -> None:
        """
        Publish TF: camera_link -> aruco_<id>
        rvec, tvec come from OpenCV ArUco
        """

        # Convert Rodrigues rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)

        # Build homogeneous transform
        T = np.eye(4)
        T[:3, :3] = R

        # Convert rotation matrix to quaternion
        quat = tf_transformations.quaternion_from_matrix(T)

        # Create TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"aruco_{marker_id}"

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

    # This function detects ArUco markers in the webcam feed and prints their IDs and positions
    def aruco_detection(self) -> None:
        # initialize a connection to the webcam and store that connection in the "cap" object
        cap = self.setup_camera()
        if cap is None:
            self.get_logger().error("Failed to open camera")
            return

        while True:
            # Capture a frame (ret is True if successful)
            ret, image = cap.read()
            if not ret:
                # logging.warning('Frame not captured; retrying...')
                # short sleep to avoid busy loop when camera fails
                Time.sleep(0.1)
                continue

            markers = self.detector.detect_markers(image)
            output = self.detector.draw_detections(image, markers)

            for m in markers:
                if m.rvec is not None and m.tvec is not None:
                    self.publish_marker_tf(m.rvec, m.tvec, m.marker_id)

            # Always show the live frame so the window remains responsive
            cv2.imshow("Detected Markers", output)

            # Process GUI events; break on 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    aruco_node = None
    try:
        aruco_node = aruco_detection_node()
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        if aruco_node is not None:
            aruco_node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        if aruco_node is not None:
            cv2.destroyAllWindows()
            aruco_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
