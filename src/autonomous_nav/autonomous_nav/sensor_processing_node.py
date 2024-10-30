import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__("sensor_processing_node")

        # Subscription to lidar data
        self.lidar_subscription = self.create_subscription(
            LaserScan, "/lidar_scan", self.lidar_callback, 10
        )

    def lidar_callback(self, msg):
        # Process lidar scan data for obstacles
        obstacles = self.detect_obstacles(msg.ranges)
        if obstacles:
            self.get_logger().info(f"Obstacles detected at ranges: {obstacles}")

    def detect_obstacles(self, scan_ranges):
        # Identify obstacles closer than 1 meter
        return [dist for dist in scan_ranges if dist < 1.0]


def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorProcessingNode()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
