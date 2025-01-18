"""
Role: This script defines the sensor processing node, responsible for processing all sensor data, 
primarily camera images, for object and obstacle detection. Data is published in a format suitable 
for other nodes.

Functionality:

- Subscribe to images and process for obstacle & object recognition with OpenCV.
- Outputs data to the Decision Making and Localization Nodes as needed.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - sensor_msgs.msg: Provides LaserScan and Imu for sensor data.
    - std_msgs.msg: Provides Float32 for processed sensor data.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32

from lib.color_codes import ColorCodes, colorStr


class SensorProcessingNode(Node):
    """
    A ROS 2 node for processing sensor data.

    This node subscribes to raw sensor data, processes it, and publishes the
    processed results to other nodes for use in decision-making and control.

    Attributes:
        lidar_sub (Subscription): A ROS 2 subscription for the "/lidar_scan" topic.
        imu_sub (Subscription): A ROS 2 subscription for the "/imu_data" topic.
        processed_pub (Publisher): A ROS 2 publisher for processed sensor data.
    """

    def __init__(self) -> None:
        super().__init__("sensor_processing_node")

        # Subscribers for raw sensor data
        self.lidar_sub = self.create_subscription(LaserScan, "/lidar_scan", self.processLidar, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu_data", self.processImu, 10)

        # Publisher for processed data
        self.processed_pub = self.create_publisher(Float32, "/processed_data", 10)

    def processLidar(self, msg: LaserScan) -> None:
        """
        Processes lidar data and publishes the result.

        Args:
            msg (LaserScan): The raw lidar data.
        """
        # Example: Calculate the minimum distance
        min_distance = min(msg.ranges)
        self.get_logger().info(f"Processed lidar data. Min distance: {min_distance}")
        self.processed_pub.publish(Float32(data=min_distance))

    def processImu(self, msg: Imu) -> None:
        """
        Processes IMU data and logs orientation.

        Args:
            msg (Imu): The raw IMU data.
        """
        orientation = msg.orientation
        self.get_logger().info(
            f"Processed IMU data. Orientation: "
            f"x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
        )


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the SensorProcessingNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    try:
        sensor_processing_node = SensorProcessingNode()
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sensor_processing_node.get_logger().info(
            colorStr("Shutting down example_node node", ColorCodes.BLUE_OK)
        )
        sensor_processing_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
