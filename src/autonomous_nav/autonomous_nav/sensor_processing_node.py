import math
import struct
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
from cv2 import aruco
from cv_bridge import CvBridge
import octomap
from octomap_msgs.msg import Octomap as OctomapMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32MultiArray

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO
from visualization_msgs.msg import MarkerArray


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data with:
      - ArUco marker detection
      - Point cloud to octomap processing
      - Custom object detection for "Hammer" + "Bottle" using YOLO World
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        self.get_logger().info("Initializing sensor_processing_node...")

        self.bridge = CvBridge()
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

        self.cloud_frame_count = 0  # Counter for point cloud frames

        # ----------------------------------------------------------------------
        # Initialize an octomap
        self.octomap_resolution = 0.05  # 5cm resolution
        self.octree = octomap.OcTree(self.octomap_resolution)

        self.max_range = 10.0  # Maximum range for point cloud processing
        self.min_range = 0.1  # Minimum range for point cloud processing
        self.sensor_origin = octomap.Point3d(0.0, 0.0, 0.0)  # Sensor origin in octomap frame

        self.camera_frame_id = "zed_camera_frame"
        self.map_frame_id = "map"
        # ----------------------------------------------------------------------
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.arucoMarkerDetection, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.processCameraInfo, 10
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.cloudCallBack, 10
        )

        self.get_logger().info("sensor_processing_node is up and running.")
        # ----------------------------------------------------------------------
        # Publishers
        self.octomap_pub = self.create_publisher(OctomapMsg, "/octomap_binary", 10)
        self.occupied_cells_pub = self.create_publisher(
            MarkerArray, "/occupied_cells_vis_array", 10
        )
        self.free_space_pub = self.create_publisher(MarkerArray, "/free_cells_vis_array", 10)

        # Timer for periodic OctoMap publishing
        self.octomap_publish_timer = self.create_timer(2.0, self.publish_octomap)

    # --------------------------------------------------------------------------
    #   processCameraInfo
    # --------------------------------------------------------------------------
    def processCameraInfo(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    # --------------------------------------------------------------------------
    #   YOLO World Object Detection
    # --------------------------------------------------------------------------
    # def yoloDetectionCallback(self, msg: Image) -> None:
    #     """
    #     Runs YOLO World detection on the camera feed.
    #     """
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert image for YOLO World detection: {e}")
    #         return

    #     # Predict with YOLO World
    #     results = self.model(frame, conf=0.3)[0]  # Adjust confidence threshold if needed

    #     # Get frame dimensions
    #     frame_h, frame_w = frame.shape[:2]

    #     # Filter results for "hammer" and "bottle"
    #     for det in results.boxes.data:
    #         x1, y1, x2, y2, confidence, class_idx = det.tolist()
    #         label = self.model.names[int(class_idx)]  # Get detected object name
    #         confidence = float(confidence)

    #         if confidence < 0.3:  # Adjust confidence threshold if needed
    #             continue

    #         # Pick color
    #         color = (255, 255, 255) if label.lower() == "bottle" else (0, 165, 255)

    #         # Draw bounding box
    #         cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    #         text = f"{label} {confidence:.2f}"
    #         cv2.putText(
    #             frame, text, (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
    #         )

    #     # Display result
    #     disp = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
    #     cv2.imshow("YOLO World Detection", disp)
    #     cv2.waitKey(1)

    # --------------------------------------------------------------------------
    #   Point Cloud Processing
    # --------------------------------------------------------------------------
    def cloudCallBack(self, msg: PointCloud2) -> None:
        try:
            # limit the processing to every 30th frame
            self.cloud_frame_count += 1
            if self.cloud_frame_count % 20 != 0:
                return
            self.get_logger().info(
                f"Processing point cloud frame {self.cloud_frame_count} for OctoMap..."
            )
            points = self.extract_all_points(msg)

            # if no points were extracted, log a warning
            if points.size == 0:
                self.get_logger().warning("No points extracted from cloud")
                return

            # filter invalid points
            finite_mask = np.isfinite(points)
            valid_rows = np.all(finite_mask, axis=1)
            valid_depth_mask = points[:, 2] > self.min_range
            range_mask = points[:, 2] < self.max_range
            combined_mask = valid_rows & valid_depth_mask & range_mask
            valid_points = points[combined_mask]

            if len(valid_points) > 0:
                self.get_logger().info(f"Updating OctoMap with {len(valid_points)} valid points...")
                self.update_octomap(valid_points, msg.header.stamp)
            else:
                self.get_logger().warning("No valid points found after filtering")
        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}")
            import traceback

            self.get_logger().error(f"Full traceback: {traceback.format_exc()}")

    def extract_all_points(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Extract all points from PointCloud2 message"""
        try:
            x_offset = y_offset = z_offset = 0
            for field in cloud_msg.fields:
                if field.name == "x":
                    x_offset = field.offset
                elif field.name == "y":
                    y_offset = field.offset
                elif field.name == "z":
                    z_offset = field.offset
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().warning("PointCloud2 does not contain x, y, z fields")
                return np.array([])
            data = cloud_msg.data

            total_points = cloud_msg.width * cloud_msg.height
            points = np.full((total_points, 3), np.nan, dtype=np.float32)

            self.get_logger().info(f"Extracting {total_points} points from PointCloud2")

            point_step = cloud_msg.point_step
            row_step = cloud_msg.row_step

            valid_count = 0
            for i in range(cloud_msg.height):
                for j in range(cloud_msg.width):
                    try:
                        point_index = i * cloud_msg.width + j
                        base_offset = i * row_step + j * point_step

                        # extract x, y, z
                        x_index = base_offset + x_offset
                        y_index = base_offset + y_offset
                        z_index = base_offset + z_offset

                        if (
                            x_index + 4 <= len(data)
                            and y_index + 4 <= len(data)
                            and z_index + 4 <= len(data)
                        ):
                            x = struct.unpack_from("f", data, x_index)[0]
                            y = struct.unpack_from("f", data, y_index)[0]
                            z = struct.unpack_from("f", data, z_index)[0]

                            points[point_index] = [x, y, z]
                            valid_count += 1
                    except struct.error as e:
                        self.get_logger().error(f"Struct error at point index {point_index}: {e}")
                        continue
            self.get_logger().info(f"Extracted {valid_count} valid points out of {total_points}")
            return points
        except Exception as e:
            self.get_logger().error(f"Error extracting points: {e}")
            return np.empty((0, 3), dtype=np.float32)

    def analyze_full_point_cloud(self, points: np.ndarray) -> None:
        """Comprehensive analysis of the full point cloud"""

        # Basic statistics
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        mean_vals = np.mean(points, axis=0)

        self.get_logger().info(f"=== FULL POINT CLOUD ANALYSIS ===")
        self.get_logger().info(f"3D Bounds:")
        self.get_logger().info(f"  X: {min_vals[0]:.2f} to {max_vals[0]:.2f}m (left/right)")
        self.get_logger().info(f"  Y: {min_vals[1]:.2f} to {max_vals[1]:.2f}m (up/down)")
        self.get_logger().info(f"  Z: {min_vals[2]:.2f} to {max_vals[2]:.2f}m (distance)")
        self.get_logger().info(
            f"Center of mass: ({mean_vals[0]:.2f}, {mean_vals[1]:.2f}, {mean_vals[2]:.2f})"
        )

        # Distance analysis
        distances = np.sqrt(np.sum(points**2, axis=1))

        close_mask = distances < 1.0
        medium_mask = (distances >= 1.0) & (distances < 3.0)
        far_mask = distances >= 3.0

        close_count = np.sum(close_mask)
        medium_count = np.sum(medium_mask)
        far_count = np.sum(far_mask)

        self.get_logger().info(f"Distance Distribution:")
        self.get_logger().info(
            f"  Close (<1m):   {close_count:5d} points ({close_count/len(points)*100:.1f}%)"
        )
        self.get_logger().info(
            f"  Medium (1-3m): {medium_count:5d} points ({medium_count/len(points)*100:.1f}%)"
        )
        self.get_logger().info(
            f"  Far (>3m):     {far_count:5d} points ({far_count/len(points)*100:.1f}%)"
        )

        # Height analysis (Y coordinate)
        ground_level = np.percentile(points[:, 1], 75)  # Assume ground is where most points are

        above_ground = points[points[:, 1] < ground_level - 0.3]  # 30cm above ground
        ground_points = points[np.abs(points[:, 1] - ground_level) < 0.3]  # Near ground level
        below_ground = points[points[:, 1] > ground_level + 0.3]  # Below ground level

        self.get_logger().info(f"Height Analysis (ground level ≈ {ground_level:.2f}m):")
        self.get_logger().info(f"  Above ground: {len(above_ground):5d} points (obstacles)")
        self.get_logger().info(f"  Ground level: {len(ground_points):5d} points")
        self.get_logger().info(f"  Below ground: {len(below_ground):5d} points")

        # Obstacle detection for navigation
        self.detect_navigation_obstacles(points)

        # Spatial density analysis
        self.analyze_spatial_density(points)

    def detect_navigation_obstacles(self, points: np.ndarray) -> None:
        """Detect obstacles relevant for robot navigation"""

        # Focus on points in front of the robot (positive Z) and at reasonable height
        forward_points = points[points[:, 2] > 0]

        if len(forward_points) == 0:
            self.get_logger().warning("No forward-facing points detected")
            return

        # Filter by height - focus on robot-level obstacles
        robot_height_mask = (forward_points[:, 1] > 0.5) & (
            forward_points[:, 1] < 2.5
        )  # 0.5m to 2.5m from camera
        obstacle_points = forward_points[robot_height_mask]

        if len(obstacle_points) > 0:
            # Find closest obstacles
            distances_2d = np.sqrt(
                obstacle_points[:, 0] ** 2 + obstacle_points[:, 2] ** 2
            )  # X-Z plane distance

            close_obstacles = obstacle_points[distances_2d < 2.0]  # Within 2 meters

            if len(close_obstacles) > 0:
                closest_idx = np.argmin(distances_2d)
                closest_obstacle = obstacle_points[closest_idx]
                closest_distance = distances_2d[closest_idx]

                self.get_logger().warn(f"=== NAVIGATION WARNING ===")
                self.get_logger().warn(
                    f"Obstacles detected: {len(close_obstacles)} points within 2m"
                )
                self.get_logger().warn(
                    f"Closest obstacle: ({closest_obstacle[0]:.2f}, {closest_obstacle[1]:.2f}, {closest_obstacle[2]:.2f})"
                )
                self.get_logger().warn(f"Distance: {closest_distance:.2f}m")

                # Analyze left vs right
                left_obstacles = close_obstacles[close_obstacles[:, 0] < -0.5]
                right_obstacles = close_obstacles[close_obstacles[:, 0] > 0.5]
                center_obstacles = close_obstacles[np.abs(close_obstacles[:, 0]) <= 0.5]

                self.get_logger().warn(f"Obstacle distribution:")
                self.get_logger().warn(f"  Left side:   {len(left_obstacles)} obstacles")
                self.get_logger().warn(f"  Center:      {len(center_obstacles)} obstacles")
                self.get_logger().warn(f"  Right side:  {len(right_obstacles)} obstacles")

                # Simple navigation recommendation
                if len(center_obstacles) > 50:  # Significant obstacle ahead
                    if len(left_obstacles) < len(right_obstacles):
                        self.get_logger().info("NAVIGATION: Recommend turning LEFT")
                    elif len(right_obstacles) < len(left_obstacles):
                        self.get_logger().info("NAVIGATION: Recommend turning RIGHT")
                    else:
                        self.get_logger().warn("NAVIGATION: STOP - obstacles on both sides")
                else:
                    self.get_logger().info("NAVIGATION: Path ahead appears clear")
            else:
                self.get_logger().info("NAVIGATION: No close obstacles detected - path clear")

    def analyze_spatial_density(self, points: np.ndarray) -> None:
        """Analyze point density in different regions"""

        # Create a simple 3D grid analysis
        x_bins = np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 5)
        z_bins = np.linspace(0.5, 5.0, 5)  # Focus on 0.5m to 5m forward

        self.get_logger().info(f"=== SPATIAL ANALYSIS ===")

        for i in range(len(z_bins) - 1):
            z_mask = (points[:, 2] >= z_bins[i]) & (points[:, 2] < z_bins[i + 1])
            z_points = points[z_mask]

            if len(z_points) > 0:
                left_count = np.sum(z_points[:, 0] < -0.5)
                center_count = np.sum(np.abs(z_points[:, 0]) <= 0.5)
                right_count = np.sum(z_points[:, 0] > 0.5)

                self.get_logger().info(
                    f"Zone {z_bins[i]:.1f}-{z_bins[i+1]:.1f}m: L={left_count:3d} C={center_count:3d} R={right_count:3d}"
                )

    # Also add this method for frame rate control

    # --------------------------------------------------------------------------
    #   octomap integration
    # --------------------------------------------------------------------------
    def update_octomap(self, points: np.ndarray, timestamp: Time) -> None:
        """Update the OctoMap with new point cloud data"""

        # Convert numpy points to OctoMap point cloud
        octomap_cloud = octomap.Pointcloud()

        for point in points:
            # Filter out ground points (assuming camera is mounted above ground)
            if point[1] < -0.5 or point[1] > 2.5:  # Skip points too low or too high
                continue

            octo_point = octomap.point3d(float(point[0]), float(point[1]), float(point[2]))
            octomap_cloud.push_back(octo_point)

        if octomap_cloud.size() == 0:
            self.get_logger().warning("No valid points for OctoMap after filtering")
            return

        # Insert the point cloud into the octree
        # The sensor origin should be updated based on robot pose if available
        self.octree.insertPointCloud(octomap_cloud, self.sensor_origin, self.max_range)

        # Update inner nodes
        self.octree.updateInnerOccupancy()

        self.get_logger().info(
            f"Updated OctoMap with {octomap_cloud.size()} points. Tree size: {self.octree.size()}"
        )

    def publish_octomap(self) -> None:
        """Publish the current OctoMap as a binary message"""
        try:
            # Create OctoMap message
            octomap_msg = OctomapMsg()
            octomap_msg.header.frame_id = self.map_frame_id
            octomap_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Serialize octree to binary
            octomap_msg.binary = True
            octomap_msg.id = "OcTree"
            octomap_msg.resolution = self.octomap_resolution
            
            # Write octree to binary stream
            s = self.octree.writeBinary()
            octomap_msg.data = list(s)
            
            self.octomap_pub.publish(octomap_msg)
            self.get_logger().info(f"Published OctoMap with {self.octree.size()} nodes")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish OctoMap: {e}")
    # --------------------------------------------------------------------------
    #   ArUco Marker Detection
    # --------------------------------------------------------------------------
    def arucoMarkerDetection(self, msg: Image) -> None:
        """
        Basic ArUco marker detection.
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warning("Camera intrinsics not received yet. Skipping frame.")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()
        aruco_detector = aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = aruco_detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected ArUco markers: {ids.flatten()}")

    # --------------------------------------------------------------------------
    #   ROS 2 Node Main
    # --------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    sensor_processing_node = None
    try:
        sensor_processing_node = SensorProcessingNode()
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                "Keyboard interrupt received. Shutting down..."
            )
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
