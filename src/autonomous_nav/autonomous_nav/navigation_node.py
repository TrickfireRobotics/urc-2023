"""
Role: This script defines the navigation node responsible for managing the navigation logic
of the rover using lat/lon input from the command line or other nodes, anchored by a
dynamically received lat/lon from /anchor_position.

Functionality:
    - Subscribes to /anchor_position (Float64MultiArray) to set the reference lat/lon anchor.
    - Subscribes to /goal_latlon (NavSatFix) to receive a latitude/longitude waypoint.
    - Converts lat/lon to local (x,y) coordinates once anchor is known.
    - Subscribes to /odometry/filtered for current pose (from robot_localization).
    - Publishes navigation status (String) on /navigation_status.
    - Publishes navigation feedback (Pose2D) on /navigation_feedback, giving
      (x_error, y_error, heading_error).
    - Clears the active waypoint when within 1 meter, or updates it on new lat/lon input.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - sensor_msgs.msg: NavSatFix for lat/lon input.
    - nav_msgs.msg: Odometry for current pose.
    - std_msgs.msg: String, Float64MultiArray for anchor and status updates.
    - geometry_msgs.msg: Pose2D for feedback.
"""

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Float64MultiArray

from typing import Optional, Tuple

from lib.color_codes import ColorCodes, colorStr


class NavigationNode(Node):
    """
    A ROS 2 node for handling robot navigation using latitude/longitude waypoints
    relative to a dynamically received anchor lat/lon.

    This node:
        - Subscribes to /anchor_position (Float64MultiArray) for [lat, lon, alt].
        - Once anchor is set, it uses this as (0,0) in the local map frame.
        - Subscribes to /goal_latlon (NavSatFix) for a new lat/lon waypoint.
        - Subscribes to /odometry/filtered (Odometry) for the current pose & orientation (yaw).
        - Publishes navigation status on /navigation_status (String).
        - Publishes navigation feedback on /navigation_feedback (Pose2D).
    """

    def __init__(self) -> None:
        super().__init__("navigation_node")

        # ---- Configuration / Parameters ----
        self.reached_threshold = 1.0  # meters
        self.earth_radius = 6371000.0  # Approx Earth radius in meters

        # ---- Anchor State (to be set from /anchor_position) ----
        self.anchor_received = False
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0

        # ---- Internal State ----
        self.active_waypoint: Optional[Tuple[float, float]] = None  # (x, y) in local map frame
        self.current_position = (0.0, 0.0)  # x, y
        self.current_yaw = 0.0

        # ---- Subscribers ----
        # Anchor subscription (transient local in GpsAnchorNode ensures we get last message)
        self.anchor_sub = self.create_subscription(
            Float64MultiArray, "/anchor_position", self.anchorCallback, 10
        )

        # Waypoint subscription
        self.latlon_sub = self.create_subscription(
            NavSatFix, "/goal_latlon", self.processLatLonGoal, 10
        )

        # Odometry subscription
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odomCallback, 10
        )

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/navigation_status", 10)
        self.feedback_pub = self.create_publisher(Pose2D, "/navigation_feedback", 10)

        # ---- Timers ----
        self.timer = self.create_timer(0.1, self.updateNavigation)  # 10 Hz

        self.get_logger().info(
            colorStr("NavigationNode (dynamic anchor) initialized", ColorCodes.BLUE_OK)
        )

    # ----------------------
    #   Subscriptions
    # ----------------------
    def anchorCallback(self, msg: Float64MultiArray) -> None:
        """
        Receives the anchor position [lat, lon, alt] from GpsAnchorNode.
        Sets our reference lat/lon for local coordinate conversion.
        """
        if len(msg.data) < 2:
            self.get_logger().warn("Received anchor message with insufficient data.")
            return

        # If anchor is already received, you can ignore or update. Here we choose to update once.
        if not self.anchor_received:
            self.ref_lat = msg.data[0]
            self.ref_lon = msg.data[1]
            if len(msg.data) >= 3:
                self.ref_alt = msg.data[2]
            self.anchor_received = True
            self.get_logger().info(
                colorStr(
                    f"Anchor received. ref_lat={self.ref_lat:.6f}, "
                    f"ref_lon={self.ref_lon:.6f}, ref_alt={self.ref_alt:.2f}",
                    ColorCodes.YELLOW_WARN,
                )
            )

    def processLatLonGoal(self, msg: NavSatFix) -> None:
        """
        Converts lat/lon from the NavSatFix message into local x,y coordinates
        *only* if anchor has been received. Otherwise, no active waypoint is set.
        """
        if not self.anchor_received:
            self.get_logger().warn("Received /goal_latlon but anchor not set yet. Ignoring goal.")
            return

        lat = msg.latitude
        lon = msg.longitude

        x, y = self.convertLatLonToXY(lat, lon)
        self.active_waypoint = (x, y)

        self.get_logger().info(
            colorStr(
                f"New lat/lon goal received: lat={lat:.6f}, lon={lon:.6f}"
                f" => local (x={x:.2f}, y={y:.2f})",
                ColorCodes.YELLOW_WARN,
            )
        )

    def odomCallback(self, msg: Odometry) -> None:
        """
        Updates current_position and current_yaw from the odometry data.
        """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ----------------------
    #   Main Navigation Logic
    # ----------------------
    def updateNavigation(self) -> None:
        """
        Periodic check of distance to the active waypoint, plus publishing
        status and feedback.
        """
        # If anchor not received, do nothing
        if not self.anchor_received:
            self.publishStatus("No anchor received; Navigation Stopped.")
            return

        # If no active waypoint
        if self.active_waypoint is None:
            self.publishStatus("No waypoint provided; Navigation Stopped.")
            return

        # Calculate distance to waypoint
        goal_x, goal_y = self.active_waypoint
        dist_to_goal = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_x, goal_y
        )

        if dist_to_goal < self.reached_threshold:
            # Reached => Publish success, clear waypoint
            self.publishStatus(f"Successfully reached waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.active_waypoint = None
            return

        # Still en route => Publish "En route..." and feedback
        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    # ----------------------
    #   Publishing Helpers
    # ----------------------
    def publishStatus(self, msg: str) -> None:
        """
        Publishes a navigation status string to /navigation_status.
        """
        self.status_pub.publish(String(data=msg))
        self.get_logger().info(colorStr(msg, ColorCodes.GREEN_OK))

    def publishFeedback(self, goal_x: float, goal_y: float) -> None:
        """
        Publishes navigation feedback (Pose2D) with (dx, dy, heading_error).
        """
        dx = goal_x - self.current_position[0]
        dy = goal_y - self.current_position[1]

        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)

        feedback = Pose2D()
        feedback.x = dx
        feedback.y = dy
        feedback.theta = heading_error
        self.feedback_pub.publish(feedback)

    # ----------------------
    #   Coordinate Conversion
    # ----------------------
    def convertLatLonToXY(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Converts latitude/longitude to an approximate local X, Y in meters
        relative to an anchor lat/lon using an equirectangular approximation.
        """
        # Convert degrees to radians
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        ref_lat_r = math.radians(self.ref_lat)
        ref_lon_r = math.radians(self.ref_lon)

        # Equirectangular approximation around the anchor
        x = (lon_r - ref_lon_r) * math.cos((lat_r + ref_lat_r) / 2.0) * self.earth_radius
        y = (lat_r - ref_lat_r) * self.earth_radius
        return x, y

    # ----------------------
    #   Utilities
    # ----------------------
    @staticmethod
    def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Returns the Euclidean distance between two points in 2D.
        """
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def quaternion_to_yaw(x_q: float, y_q: float, z_q: float, w_q: float) -> float:
        """
        Converts a quaternion into a yaw angle in radians (Z-axis rotation).
        """
        siny_cosp = 2.0 * (w_q * z_q + x_q * y_q)
        cosy_cosp = 1.0 - 2.0 * (y_q * y_q + z_q * z_q)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalizes an angle (in radians) into the range (-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the NavigationNode.
    """
    rclpy.init(args=args)
    navigation_node = None
    try:
        navigation_node = NavigationNode()
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if navigation_node is not None:
            navigation_node.get_logger().info(
                colorStr("External shutdown request received", ColorCodes.BLUE_OK)
            )
    finally:
        if navigation_node is not None:
            navigation_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
