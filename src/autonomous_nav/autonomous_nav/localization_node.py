"""
Role: This script tracks the rover's position using GNSS, IMU, or other localization sensors.

Functionality: Integrates data from the sensor suite, excluding object detection camera data, to
track rover position & orientation.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - geometry_msgs.msg: Provides PoseStamped messages for robot localization.
"""

import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

from lib.color_codes import ColorCodes, colorStr


class LocalizationNode(Node):
    """
    A ROS 2 node for handling robot localization.

    This node processes sensor data to estimate the robot's pose (position and
    orientation) in the environment and publishes the pose to a specified topic.

    Attributes:
        pose_pub (Publisher): A ROS 2 publisher for the "/robot_pose" topic.
        current_position (tuple): Stores the latest GPS position (latitude, longitude, altitude).
        current_orientation (tuple): Stores the latest IMU orientation (x, y, z, w).
    """

    def __init__(self) -> None:
        super().__init__("localization_node")

        # Subscriptions for GPS and IMU data
        self.gps_subscription = self.create_subscription(
            NavSatFix, "/gps/fix", self.gpsCallback, 10
        )
        self.imu_subscription = self.create_subscription(Imu, "/imu/data", self.imuCallback, 10)

        # Publisher for the robot's pose
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # Timer to periodically publish the robot's pose
        self.timer = self.create_timer(0.1, self.publishPose)

        # Initialize the robot's pose
        self.current_pose = PoseStamped()
        self.current_position = (0.0, 0.0, 0.0)
        self.current_orientation = (0.0, 0.0, 0.0, 1.0)

    def gpsCallback(self, msg: NavSatFix) -> None:
        """Processes GPS data to update current position."""
        self.current_position = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(f"Updated GPS position: {self.current_position}")
        self.publishPose()

    def imuCallback(self, msg: Imu) -> None:
        """Processes IMU data to update current orientation."""
        self.current_orientation = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.get_logger().info(f"Updated orientation: {self.current_orientation}")
        self.publishPose()

    def publishPose(self) -> None:
        """
        Publishes the robot's current pose to the "/robot_pose" topic.
        """
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = "map"

        # Simulated movement (this would typically come from sensor data)
        self.current_pose.pose.position.x = self.current_position[0]
        self.current_pose.pose.position.y = self.current_position[1]
        self.current_pose.pose.position.z = self.current_position[2]

        self.current_pose.pose.orientation.x = self.current_orientation[0]
        self.current_pose.pose.orientation.y = self.current_orientation[1]
        self.current_pose.pose.orientation.z = self.current_orientation[2]
        self.current_pose.pose.orientation.w = self.current_orientation[3]

        # Publish the pose
        self.pose_pub.publish(self.current_pose)
        # self.get_logger().info(f"Published pose: {self.current_pose}")

    def updatePose(self, pose: PoseStamped) -> None:
        """
        Updates the robot's current pose based on sensor data.

        Args:
            pose (PoseStamped): The new pose of the robot.
        """
        self.current_pose = pose
        self.get_logger().info(f"Updated pose to: {pose}")


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the LocalizationNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    try:
        localization_node = LocalizationNode()
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        localization_node.get_logger().info(
            colorStr("Shutting down localization node", ColorCodes.BLUE_OK)
        )
        localization_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
