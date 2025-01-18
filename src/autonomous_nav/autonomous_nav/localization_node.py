"""
Role: This script tracks the rovers position using GNSS, IMU, or other localization sensors.

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

from lib.color_codes import ColorCodes, colorStr


class LocalizationNode(Node):
    """
    A ROS 2 node for handling robot localization.

    This node processes sensor data to estimate the robot's pose (position and
    orientation) in the environment and publishes the pose to a specified topic.

    Attributes:
        pose_pub (Publisher): A ROS 2 publisher for the "/robot_pose" topic.
    """

    def __init__(self) -> None:
        super().__init__("localization_node")

        # Publisher for the robot's pose
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # Timer to periodically publish the robot's pose
        self.timer = self.create_timer(0.1, self.publishPose)

        # Initialize the robot's pose
        self.current_pose = PoseStamped()

    def publishPose(self) -> None:
        """
        Publishes the robot's current pose to the "/robot_pose" topic.
        """
        # Example: Update the robot's pose (this would typically come from sensors)
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.position.x += 0.01  # Simulate movement
        self.current_pose.pose.position.y += 0.01
        self.current_pose.pose.orientation.w = 1.0

        # Publish the pose
        self.pose_pub.publish(self.current_pose)
        self.get_logger().info(f"Published pose: {self.current_pose}")

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
            colorStr("Shutting down example_node node", ColorCodes.BLUE_OK)
        )
        localization_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
