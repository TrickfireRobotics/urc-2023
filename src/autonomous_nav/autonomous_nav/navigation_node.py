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
        - Subscribes to /anchor_position (Float64MultiArray).
        - Subscribes to /goal_latlon (NavSatFix) for a new lat/lon waypoint.
        - Subscribes to /odometry/filtered (Odometry) for current pose.
        - Publishes /navigation_status (String) and /navigation_feedback (Pose2D).
    """

    def __init__(self) -> None:
        super().__init__("navigation_node")

        # ---- Configuration / Parameters ----
        self.reached_threshold = 1.0  # meters
        self.earth_radius = 6371000.0  # Approx Earth radius in meters

        # ---- Anchor State (from /anchor_position) ----
        self.anchor_received = False
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0

        # ---- Internal State ----
        self.active_waypoint: Optional[Tuple[float, float]] = None
        self.current_position = (0.0, 0.0)  # x, y
        self.current_yaw = 0.0

        # ---- Subscribers ----
        self.anchor_sub = self.create_subscription(
            Float64MultiArray, "/anchor_position", self.anchorCallback, 10
        )

        self.latlon_sub = self.create_subscription(
            NavSatFix, "/goal_latlon", self.processLatLonGoal, 10
        )

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
        Receives the anchor position [lat, lon, alt].
        """
        if len(msg.data) < 2:
            self.get_logger().warn("Received anchor message with insufficient data.")
            return

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
                    ColorCodes.BLUE_OK,
                )
            )

    def processLatLonGoal(self, msg: NavSatFix) -> None:
        """
        Converts lat/lon from the NavSatFix message into local x,y coordinates
        only if anchor is known. Otherwise, do nothing.
        """
        if not self.anchor_received:
            self.get_logger().warn("Received /goal_latlon but anchor not set yet.")
            return

        lat = msg.latitude
        lon = msg.longitude

        x, y = self.convertLatLonToXY(lat, lon)
        self.active_waypoint = (x, y)
        self.get_logger().info(
            colorStr(
                f"New lat/lon goal received: lat={lat:.6f}, lon={lon:.6f} => (x={x:.2f}, y={y:.2f})",
                ColorCodes.BLUE_OK,
            )
        )

    def odomCallback(self, msg: Odometry) -> None:
        """
        Updates self.current_position and self.current_yaw from odometry.
        """
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    # ----------------------
    #   Main Navigation Logic
    # ----------------------
    def updateNavigation(self) -> None:
        # If anchor not received, publish status but do not navigate
        if not self.anchor_received:
            self.publishStatus("No anchor received; Navigation Stopped.")
            return

        # If no active waypoint
        if self.active_waypoint is None:
            self.publishStatus("No waypoint provided; Navigation Stopped.")
            return

        # Compute distance to the waypoint
        goal_x, goal_y = self.active_waypoint
        dist_to_goal = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_x, goal_y
        )

        if dist_to_goal < self.reached_threshold:
            # Reached => Publish success, clear waypoint
            self.publishStatus(f"Successfully reached waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.active_waypoint = None
            return

        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    # ----------------------
    #   Publishing Helpers
    # ----------------------
    def publishStatus(self, msg: str) -> None:
        self.status_pub.publish(String(data=msg))
        # self.get_logger().info(colorStr(msg, ColorCodes.GREEN_OK))

    def publishFeedback(self, gx: float, gy: float) -> None:
        dx = gx - self.current_position[0]
        dy = gy - self.current_position[1]
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
        lat_r = math.radians(lat)
        lon_r = math.radians(lon)
        ref_lat_r = math.radians(self.ref_lat)
        ref_lon_r = math.radians(self.ref_lon)

        x = (lon_r - ref_lon_r) * math.cos((lat_r + ref_lat_r) / 2.0) * self.earth_radius
        y = (lat_r - ref_lat_r) * self.earth_radius
        return (x, y)

    # ----------------------
    #   Utilities
    # ----------------------
    @staticmethod
    def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def quaternion_to_yaw(x_q: float, y_q: float, z_q: float, w_q: float) -> float:
        siny_cosp = 2.0 * (w_q * z_q + x_q * y_q)
        cosy_cosp = 1.0 - 2.0 * (y_q * y_q + z_q * z_q)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.get_logger().info(colorStr("External shutdown request received", ColorCodes.BLUE_OK))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
