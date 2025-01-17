""" 
Localization node responsible for keeping track of the rover's current location, 
determined by sensor fusion
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix


class LocalizationNode(Node):
    """Node for localizing the rover using GPS and IMU data"""

    def __init__(self) -> None:
        super().__init__("localization_node")

        # Subscriptions for GPS and IMU data
        self.gps_subscription = self.create_subscription(
            NavSatFix, "/gps/fix", self.gpsCallback, 10
        )
        self.imu_subscription = self.create_subscription(Imu, "/imu/data", self.imuCallback, 10)

        # Variables to store current position and orientation
        self.current_position = None
        self.current_orientation = None

        # Publisher for combined localization data (position + orientation)
        self.pose_publisher = self.create_publisher(Pose, "/localization/pose", 10)

    def gpsCallback(self, msg: NavSatFix) -> None:
        """Processes GPS data to update current position."""
        self.current_position = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(f"Updated GPS position: {self.current_position}")
        self.publish_pose()

    def imuCallback(self, msg: Imu) -> None:
        """Processes IMU data to update current orientation."""
        self.current_orientation = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,r
        )
        self.get_logger().info(f"Updated orientation: {self.current_orientation}")
        self.publish_pose()

    def publish_pose(self):
        """Publishes current position and orientation if both are available."""
        if self.current_position and self.current_orientation:
            pose = Pose()
            # Assign position (latitude and longitude as x and y for simplicity;
            # altitude as z)
            pose.position.x, pose.position.y, pose.position.z = self.current_position
            # Assign orientation (quaternion)
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = self.current_orientation
            self.pose_publisher.publish(pose)
            self.get_logger().info("Published updated pose.")


def main(args=None):
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    try:
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
