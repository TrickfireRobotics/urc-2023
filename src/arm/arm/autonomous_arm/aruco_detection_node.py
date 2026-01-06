# This node takes in Aruco points relative to the camera, transforms them to the robots base frame, and publishes them for use in the arm control node
import math
import struct
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
import tf_transformations
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Float32MultiArray, Header
from tf2_ros import TransformBroadcaster


# TODO: get the camera matrix and distortion coefficients from a subscription instead of hardcoding them
# TODO: publish the Aruco points in the robots base frame to a topic
class aruco_detection_node(Node):
    def __init__(self) -> None:
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        # get the camera_matrix, dist_coeffs from the camera calibration
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))
        # get the marker length from the camera calibration
        self.marker_length = 0.0529
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
        markerImage = np.zeros((200, 200), dtype=np.uint8)
        cv2.aruco.generateImageMarker(self.aruco_dict, 23, 200, markerImage, 1)
        cv2.imwrite("marker23.png", markerImage)
        # Create the ArUco detector
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        while True:
            # Capture a frame (ret is True if successful)
            ret, image = cap.read()
            if not ret:
                # logging.warning('Frame not captured; retrying...')
                # short sleep to avoid busy loop when camera fails
                Time.sleep(0.1)
                continue

            # Convert to grayscale and detect markers
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            # Draw markers and IDs when found
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(image, corners, ids)
                for i, corner in enumerate(corners):
                    pts = corner[0]
                    cx = int(pts[:, 0].mean())
                    cy = int(pts[:, 1].mean())
                    cv2.putText(
                        image,
                        f"ID:{int(ids[i])}",
                        (cx - 20, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

            # Find pose of each marker
            rvecs, tvecs, _objpoints = my_estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            for i in range(len(rvecs)):
                cv2.drawFrameAxes(
                    image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvecs[i],
                    tvecs[i],
                    self.marker_length,
                )
                # logging.info(f'Marker ID: {ids[i][0]}, rvec: {rvecs[i].flatten()}, tvec: {tvecs[i].flatten()}')
            # Always show the live frame so the window remains responsive
            cv2.imshow("Detected Markers", image)

            # Process GUI events; break on 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()


def my_estimatePoseSingleMarkers(
    corners: list[np.ndarray],
    marker_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray], list]:
    """
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    """
    marker_points = np.array(
        [
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0],
        ],
        dtype=np.float32,
    )
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, camera_matrix, dist_coeffs, None, None)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


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
