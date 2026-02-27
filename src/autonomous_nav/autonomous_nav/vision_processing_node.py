#!/home/trickfire/autonomous-nav-vision/urc-2023/my_venv/bin/python3
# use virtual enviroment
import struct
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
import torch  # for switching to cpu
from cv2 import aruco
from cv_bridge import CvBridge

# from std_msgs.msg import Bool, Float32MultiArray, Header
from geometry_msgs.msg import Vector3

# added Navigate to pose to support logic control (activating aruco and obj search)
from nav2_msgs.action import NavigateToPose

# from octomap_msgs.msg import Octomap as OctomapMsg
from rclpy.node import Node

# from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image  # , PointCloud2, PointField
from std_msgs.msg import Float32

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO

# defined in src\custom_interfaces\msg\Aruco.msg
from custom_interfaces.msg import Aruco
from lib.color_codes import ColorCodes, colorStr

# from visualization_msgs.msg import MarkerArray


class VisionProcessingNode(Node):
    """
    A ROS 2 node for processing camera data with:
      - ArUco marker detection
      - Custom object detection for "Hammer" + "Bottle" using YOLO World
    """

    def __init__(self) -> None:
        super().__init__("vision_processing_node")

        self.get_logger().info(
            colorStr("Initializing vision_processing_node...", ColorCodes.BLUE_OK)
        )
        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        # Spin search control variable (should be written better later but this is a quick solution to only activate vision processing when we are at the goal location)
        self.enable_spin = False
        self.found_aruco = False
        self.found_object = False
        # Spin search timer setup
        self.frame_counter = 0
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz timer
        self.spin_search_interval = 30  # every 30 ticks = every 3 seconds

        # Load YOLO World model
        try:
            # "yolo26x.pt" for more accuracy, if l isn't good enough
            # curl -L -o yoloe-26x-seg.pt https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26x-seg.pt
            # tried hard coding path to yolo model
            self.model = YOLO("/home/trickfire/autonomous-nav-vision-urc/urc-2023/yoloe-26x-seg.pt")
            self.model.set_classes(["mallet", "rockhammer", "hammer", "bottle"])
            # self.get_logger().info("Trained YOLO World model loaded successfully.")
            self.get_logger().info(
                colorStr("YOLO 26.l model loaded successfully.", ColorCodes.BLUE_OK)
            )
        except:
            self.get_logger().info(
                colorStr(
                    "Could not find new yolo model (yolo26l.pt), falling back on default model (yolov8l-world.pt)"
                    + torch.__version__,
                    ColorCodes.WARNING_YELLOW,
                )
            )
            self.model = YOLO("yolov8l-world.pt")
            self.get_logger().info(
                colorStr("Default YOLO World model loaded successfully.", ColorCodes.BLUE_OK)
            )

        # use cpu or gpu
        self.get_logger().info(colorStr("Torch Version:" + torch.__version__, ColorCodes.BLUE_OK))
        self.get_logger().info(
            colorStr("Torch Version Cuda:" + torch.version.cuda, ColorCodes.BLUE_OK)
        )
        self.get_logger().info(
            colorStr("CUDA Available:" + str(torch.cuda.is_available()), ColorCodes.BLUE_OK)
        )
        # Get GPU details
        # disable gpu for current tests
        self.get_logger().info(
            colorStr("GPU Unavailable, switching to cpu", ColorCodes.WARNING_YELLOW)
        )
        self.model.to("cpu")
        self.get_logger().info(colorStr("Running on cpu", ColorCodes.BLUE_OK))
        """
        if torch.cuda.is_available():
            self.get_logger().info(colorStr("GPU Name:"+str(torch.cuda.get_device_name(0)), ColorCodes.GREEN_OK))
            self.model.to("cuda")
        else:
            self.get_logger().info(colorStr("GPU Unavailable, switching to cpu", ColorCodes.WARNING_YELLOW))
            self.model.to("cpu")
            self.get_logger().info(colorStr("Running on cpu", ColorCodes.BLUE_OK))
        """
        self.camera_frame_id = "zed_camera_frame"
        self.map_frame_id = "map"

        # Aruco detector setup
        self.aruco_dict = aruco.getPredefinedDictionary(
            aruco.DICT_6X6_250
        )  # we only detect 6x6 markers
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
        self.navigation_status = self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._nav_feedback_callback,
            10,
        )

        # ----------------------------------------------------------------------
        # Publishers
        self.object_detection_pub = self.create_publisher(Image, "/object_detection_image", 10)

        self.aruco_detection_image_pub = self.create_publisher(Image, "/aruco_detection_image", 10)

        # emitted whenever we see a aruco marker
        self.aruco_detection_pub = self.create_publisher(Aruco, "/aruco_detection", 10)

        self.get_logger().info(
            colorStr("vision_processing_node is up and running.", ColorCodes.GREEN_OK)
        )
        # velocity publisher for the spin search (Please move this by 3/15/2026 it shouldnt really go here)
        self.right_wheel_pub = self.create_publisher(Float32, "/right_wheel_velocity", 10)
        self.left_wheel_pub = self.create_publisher(Float32, "/left_wheel_velocity", 10)

    def _nav_feedback_callback(self, msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        """
        Callback to monitor navigation status. Sets reached_goal to True when the robot reaches its goal.
        """
        if msg is None:
            self.get_logger().error("Received None message in navigation feedback callback")
            return

        # Check if the status indicates that the goal has been reached
        if msg.status.status == 3:  # Status code 3 typically indicates success
            self.get_logger().info(
                colorStr(
                    "Navigation goal reached! Activating vision processing.", ColorCodes.GREEN_OK
                )
            )
            self.enable_spin = True

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
        if msg is None:
            self.get_logger().error("Received None message in combinedCallback")
            return
        if self.enable_spin:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")
                return
            # maybe put a sleep or logic gate so these functions switch off every so many frames to avoid 2 functions/frame
            self.arucoMarkerDetection(frame)

            # This resizes the frame for yolo, if its not accurate enough maybe increase the size
            resized = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            self.yoloDetectionCallback(resized)
            # every 3 seconds run spin search to look for objects, this is a placeholder and can be replaced with more sophisticated search patterns
            # time.sleep(3)

    # --------------------------------------------------------------------------
    #   YOLO World Object Detection
    # --------------------------------------------------------------------------
    def yoloDetectionCallback(self, msg: np.ndarray) -> None:
        """
        Runs YOLO World detection on the camera feed.
        """
        frame = msg

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
            text = f"{confidence:.2f}"  # f"{label} {confidence:.2f}"
            cv2.putText(
                frame, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
            )
            self.found_object = True
            self.get_logger().info(
                colorStr(f"Detected {label} with confidence {confidence:.2f}", ColorCodes.BLUE_OK)
            )
            if self.found_aruco:
                self.get_logger().info(
                    colorStr(
                        f"Object and ArUco marker detected! Stopping spin search.",
                        ColorCodes.GREEN_OK,
                    )
                )
                self.enable_spin = False  # stop spin search once we have found an object and seen an aruco marker, this is a placeholder condition and can be replaced with better logic later

        # publish results to view with rviz
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
            self.get_logger().info(
                colorStr(
                    "Camera intrinsics not received yet. Skipping frame.", ColorCodes.WARNING_YELLOW
                )
            )
            return

        # cv_image isn't defined so I will assume it is mislabled frame
        cv_image = frame

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.aruco_detector.detectMarkers(gray_image)

        if ids is None or len(ids) == 0:
            return

        self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

        marker_length = 0.20  # Marker size in meters
        half_size = marker_length / 2.0

        obj_points = np.array(
            [
                [-half_size, half_size, 0],
                [half_size, half_size, 0],
                [half_size, -half_size, 0],
                [-half_size, -half_size, 0],
            ],
            dtype=np.float32,
        )

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
            rvec = rvec.reshape(3)

            distance = float(np.linalg.norm(tvec))

            self.get_logger().info(
                colorStr(
                    f"Marker ID: {int(marker_id)}, Distance: {distance:.2f} m, tvec: {tvec}, rvec: {rvec}",
                    ColorCodes.BLUE_OK,
                )
            )

            # int, float, vec3, vec3
            # emit the aruco marker id,distance,tvec,rvec
            # tvec is transform vector aka position, rvec is rotation vector

            aruco_message = Aruco()
            aruco_message.id = int(marker_id)
            aruco_message.distance = float(distance)

            tvec_out = Vector3()
            tvec_out.x = tvec[0]
            tvec_out.y = tvec[1]
            tvec_out.z = tvec[2]

            rvec_out = Vector3()
            rvec_out.x = rvec[0]
            rvec_out.y = rvec[1]
            rvec_out.z = rvec[2]

            aruco_message.transform_vec = tvec_out
            aruco_message.rotation_vec = rvec_out
            self.aruco_detection_pub.publish(aruco_message)

            # draw a axes showing the aruco marker detected
            cv2.drawFrameAxes(
                cv_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                marker_length / 2.0,
            )
            self.found_aruco = True
            if self.found_object and self.found_aruco:
                self.get_logger().info(
                    colorStr(
                        f"Object and ArUco marker detected! Stopping spin search.",
                        ColorCodes.GREEN_OK,
                    )
                )
                self.enable_spin = False  # stop spin search once we have found an object and seen an aruco marker, this is a placeholder condition and can be replaced with better logic later

        # return the finished image to rvis to see what aruco markers are being detected
        # self.get_logger().info(colorStr(f"Publishing aruco image!",ColorCodes.GREEN_OK))
        image_message = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.aruco_detection_image_pub.publish(image_message)

    # --------------------------------------------------------------------------
    #   Search Pattern (Spin in Place)
    # --------------------------------------------------------------------------
    def spinSearch(self) -> None:
        """
        A simple search pattern that spins the robot in place to look for objects.
        This is a placeholder and can be replaced with a more sophisticated search pattern if needed.
        """
        self.get_logger().info(
            colorStr("Spinning in place to search for objects...", ColorCodes.BLUE_OK)
        )
        spin_velocity = 0.3  # Adjust as needed for your robot
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = spin_velocity
        right_msg.data = -spin_velocity
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)

    # --------------------------------------------------------------------------
    #   Tick function to manage search pattern timing
    # --------------------------------------------------------------------------
    def _tick(self) -> None:
        """
        Tick function called by the timer to manage the timing of the search pattern.
        """
        if self.enable_spin:
            self.frame_counter += 1
            if self.frame_counter >= self.spin_search_interval:
                self.spinSearch()
                self.frame_counter = 0

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
            vision_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
