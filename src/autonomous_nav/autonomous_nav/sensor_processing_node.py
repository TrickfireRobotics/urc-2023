import math
import struct
import sys
from typing import Optional

import cv2  # pylint: disable=no-member
import numpy as np
import rclpy
from cv2 import aruco
from cv_bridge import CvBridge
from octomap_msgs.msg import Octomap as OctomapMsg
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, Float32MultiArray, Header

# Install: pip install "ultralytics>=8.1.0" "torch>=1.8"
from ultralytics import YOLO
from visualization_msgs.msg import MarkerArray


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data with:
      - Point cloud to octomap processing
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        self.get_logger().info("Initializing sensor_processing_node...")


        self.cloud_frame_count = 0  # Counter for point cloud frames

        self.max_range = 10.0  # Maximum range for point cloud processing
        self.min_range = 0.1  # Minimum range for point cloud processing

        self.camera_frame_id = "zed_camera_frame"
        self.map_frame_id = "map"
        # ----------------------------------------------------------------------
        # Subscriptions
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.cloudCallBack, 10
        )

        # ----------------------------------------------------------------------
        # Publishers
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2, "/filtered_point_cloud", 10
        )
        
        self.get_logger().info("sensor_processing_node is up and running.")
        self.get_logger().info("Publishing filtered point clouds to /filtered_point_cloud")
        self.get_logger().info("Start octomap_server with: ros2 run octomap_server octomap_server_node --ros-args -r cloud_in:=/filtered_point_cloud -p resolution:=0.05")

    # --------------------------------------------------------------------------
    #   Point Cloud Processing
    # --------------------------------------------------------------------------
    def cloudCallBack(self, msg: PointCloud2) -> None:
        try:
            # limit the processing to every 30th frame
            self.cloud_frame_count += 1
            if self.cloud_frame_count % 20 != 0:
                return
            #self.get_logger().info(f"Processing point cloud frame {self.cloud_frame_count}")
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
               # self.get_logger().info(f"Publishing {len(valid_points)} valid points...")
                self.publish_filtered_cloud(valid_points, msg.header)
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

            # self.get_logger().info(f"Extracting {total_points} points from PointCloud2")

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
            # self.get_logger().info(f"Extracted {valid_count} valid points out of {total_points}")
            return points
        except Exception as e:
            self.get_logger().error(f"Error extracting points: {e}")
            return np.empty((0, 3), dtype=np.float32)

    def publish_filtered_cloud(self, points: np.ndarray, original_header: Header) -> None:
        """Publish filtered point cloud"""
        try:
            cloud_msg = PointCloud2()
            cloud_msg.header = original_header
            cloud_msg.height = 1
            cloud_msg.width = len(points)
            cloud_msg.is_dense = False
            cloud_msg.is_bigendian = False

            cloud_msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud_msg.point_step = 12  # 3 fields * 4 bytes each
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

            cloud_msg.data = points.astype(np.float32).tobytes()

            self.filtered_cloud_pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish filtered cloud: {e}")
            

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
