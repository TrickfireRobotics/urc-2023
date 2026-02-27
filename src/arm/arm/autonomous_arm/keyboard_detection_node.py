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
