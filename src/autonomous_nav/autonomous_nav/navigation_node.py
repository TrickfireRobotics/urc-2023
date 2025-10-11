import math
import sys
from queue import Queue
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String

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
        self.get_logger().info(f"Navigation Node has been started successfully")

        # ---- Configuration / Parameters ----
        self.reached_threshold = 1.0  # meters
        self.earth_radius = 6371000.0  # Approx Earth radius in meters

        # ---- Anchor State (from /anchor_position) ----
        self.anchor_received = False
        self.ref_lat = 0.0
        self.ref_lon = 0.0
        self.ref_alt = 0.0
        self.start_lat = 0.0
        self.start_lon = 0.0
        self.start_alt = 0.0

        # ---- Internal State ----
        self.active_waypoint: Optional[Tuple[float, float]] = None
        self.current_position = (0.0, 0.0)  # x, y
        self.current_yaw = 0.0
        self.current_global_yaw = 0.0
        self.current_lat = 0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.end_goal_waypoint: Tuple[float, float]
        self.path: Path
        self.global_costmap: OccupancyGrid
        # ---- Subscribers ----
        # latitude, longitude, altitude
        self.global_costmap_subscription = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.costmap_callback, 10
        )

        self.anchor_sub = self.create_subscription(
            Float64MultiArray, "/anchor_position", self.anchorCallback, 10
        )

        self.latlon_sub = self.create_subscription(
            NavSatFix, "/goal_latlon", self.processLatLonGoal, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odomCallback, 10
        )
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 10)

        # TODO subscribe to the cost map right here

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/navigation_status", 10)
        self.feedback_pub = self.create_publisher(Pose2D, "/navigation_feedback", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
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

    def gpsCallback(self, msg: NavSatFix) -> None:
        if not self.anchor_received:
            return
        if msg.status.status >= 0 and self.current_lat == 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_alt = msg.altitude
            self.determine_global_yaw()
        # If fix is valid (>= 0 => at least a standard fix)
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_alt = msg.altitude
        self.get_logger().info(
            colorStr(
                f"current position received. current_lat={self.current_lat:.6f}, "
                f"current_lon={self.current_lon:.6f}, current_alt={self.current_alt:.2f}",
                ColorCodes.BLUE_OK,
            )
        )

    def determine_global_yaw(self) -> None:
        x = math.cos(math.radians(self.current_lat)) * math.sin(
            math.radians(self.current_lon) - math.radians(self.ref_lon)
        )
        y = math.cos(math.radians(self.ref_lat)) * math.sin(
            math.radians(self.current_lat)
        ) - math.sin(math.radians(self.ref_lat)) * math.cos(
            math.radians(self.current_lat)
        ) * math.cos(
            math.radians(self.current_lon) - math.radians(self.ref_lon)
        )
        result = math.atan2(x, y)
        result = result * (math.pi / 180)
        result = (result + 360) % 360
        self.current_global_yaw = result

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
        # TODO these two variables track the same thing, fix it
        self.active_waypoint = (x, y)
        self.end_goal_waypoint = (x, y)
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
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )  # RELATIVE TO FRAME
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        self.get_logger().info(f"Received costmap: {msg.info.width} x {msg.info.height}")
        self.global_costmap = msg
        # check first 10 cells
        print(msg.data[:10])

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
            # plan a set of waypoints using a queue
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
        else:
            self.planPath(self.global_costmap)
        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    def planPath(
        self,
        grid: OccupancyGrid,
    ) -> None:
        path_radius = 5
        grid_height = grid.info.height
        grid_width = grid.info.width
        grid_origin = grid.info.origin  # global coordinates of origin
        """
        This algorithm looks at the global occupancy grid in order to plan a path through it for the rover using an A* style search algorithm. 
        """
        # attain current position within costmap
        current_index = self.position_to_index(grid, self.current_position)
        # gather points within a certain radius
        # TODO make target collect radius return only the indicies
        # TODO filter out unkmow positions?
        search_radius: int = 5
        target_area: list[Tuple[int, int]] = self.collect_radius(
            grid, current_index, search_radius
        )  # list[Tuple[int,int]] corresponding to the index within the occupancy grid and the cost of that point in the index
        minmium_cost = 1000.0
        minimum_position: Tuple[float, float] = (
            self.current_position
        )  # the lowest cost position (AKA the position we will add next)
        while (
            self.distance_2d(
                minimum_position[0],
                minimum_position[1],
                self.end_goal_waypoint[0],
                self.end_goal_waypoint[1],
            )
            > 1
        ):
            # choose point with the lowest value within target area
            for item in target_area:
                item_position = self.index_to_position(
                    grid, item[1]
                )  # the x,y position of a point currently in the target area
                item_cost = item[0] + self.distance_2d(
                    item_position[0],
                    item_position[1],
                    self.end_goal_waypoint[0],
                    self.end_goal_waypoint[1],
                )  # item cost is the cost from the occupancy grid (item[1]) + distance to the goal
                if item_cost < minmium_cost and item_cost != 100:
                    minimum_position = item_position
                    target_area = self.collect_radius(
                        grid, item[1], 5
                    )  # collect everything within a 2.5 meter radius of the new minimum point
                    self.append_path(
                        item_position[0], item_position[1]
                    )  # add this new minmum cost point to our path
                    if item_cost == -1:
                        self.publishStatus(
                            f"position ({item_position[0]:.2f}, {item_position[1]:.2f}) has been chosen as unknown)"
                        )
                else:
                    self.publishStatus(
                        f"position ({item_position[0]:.2f}, {item_position[1]:.2f}) has cost of ({item_cost})"
                    )
        self.path_pub.publish(self.path)

        # add that to the queue until you find the goal position (use an if statement to check of the heuristic is 0 and if so, choose it, send to coord, and break)

    def append_path(self, x: float, y: float) -> None:
        self.path.header.frame_id = "map"
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.path.poses.append(pose)

    def collect_radius(
        self, grid: OccupancyGrid, current_index: int, radius: int
    ) -> list[Tuple[int, int]]:
        points_in_radius: list[Tuple[int, int]] = []
        row_width = grid.info.width
        num_rows = int(radius / grid.info.resolution)
        starting_index = int(
            current_index - ((grid.info.width * (radius / 2) / grid.info.resolution))
        )
        ending_index = int(
            current_index + ((grid.info.width * (radius / 2) / grid.info.resolution))
        )
        for i in range(0, num_rows):
            for j in range(0, row_width):
                index = int(starting_index + (i * row_width) + j)
                if index < len(grid.data):
                    data = int(grid.data[index])
                    points_in_radius.append((data, index))
        return points_in_radius

    def position_to_index(self, grid: OccupancyGrid, current_position: Tuple[float, float]) -> int:
        # this function takes in the occupancy grid and an index within in it, and returns the map coordinates of that point
        row = int((current_position[1] - grid.info.origin.position.y) / grid.info.resolution)
        column = int((current_position[0] - grid.info.origin.position.x) / grid.info.resolution)
        current_index = int((row * grid.info.width) + column)
        return current_index

    def index_to_position(self, grid: OccupancyGrid, target_index: int) -> Tuple[float, float]:
        row = target_index // grid.info.resolution
        col = target_index % grid.info.resolution
        x_position = grid.info.origin.position.x + (col + 0.5) * grid.info.resolution
        y_position = grid.info.origin.position.y + (row + 0.5) * grid.info.resolution
        return (x_position, y_position)

    def turnTowardGoal(self, goal_Location: Tuple[float, float]) -> None:
        a = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_Location[0], goal_Location[1]
        )
        b = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        c = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        temp = (a**2 - c**2 - b**2) / (-2 * b * c)
        turn_angle = math.degrees(math.acos(temp))
        # read from current pose and anchor position
        # do the same calculation using measurmentts relative to the anchor
        # turn left or right that number of degrees

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
    except Exception as e:
        node.get_logger().error(f"Exception during spin: {e}")
    # except ExternalShutdownException:
    # node.get_logger().info(colorStr("External shutdown request received", ColorCodes.BLUE_OK))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
