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

# from lib.color_codes import ColorCodes, colorStr


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
        self.get_logger().info(f"Navigation Node has been started successfully...")

        # ---- Configuration / Parameters ----
        self.reached_threshold = 0.05  # meters
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
        self.active_waypoint: Optional[Tuple[float, float]] = (3, 0)
        self.current_position = (0.0, 0.0)  # x, y
        self.current_yaw = 0.0
        self.current_global_yaw = 0.0
        self.current_lat = 0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.end_goal_waypoint: Tuple[float, float] = (3, 0)
        self.path: Path = Path()
        self.global_costmap: Optional[OccupancyGrid] = None
        self.end_goal_index: int = 0
        self.get_logger().info(f"Navigation Node has initialized first arguments")
        # ---- Subscribers ----
        # latitude, longitude, altitude
        self.global_costmap_subscription = self.create_subscription(
            OccupancyGrid, "/projected_map", self.costmap_callback, 10
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

        # ---- Publishers ----
        self.status_pub = self.create_publisher(String, "/navigation_status", 10)
        self.feedback_pub = self.create_publisher(Pose2D, "/navigation_feedback", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.pos_pub = self.create_publisher(Pose2D, "/pos", 10)
        # ---- Timers ----
        self.timer = self.create_timer(5, self.updateNavigation)  # 0.2 Hz

        """self.get_logger().info(
            colorStr("NavigationNode (dynamic anchor) initialized", ColorCodes.BLUE_OK)
        )"""

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
                """colorStr(
                    f"Anchor received. ref_lat={self.ref_lat:.6f}, "
                    f"ref_lon={self.ref_lon:.6f}, ref_alt={self.ref_alt:.2f}",
                    ColorCodes.BLUE_OK,
                )"""
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
            """colorStr(
                f"current position received. current_lat={self.current_lat:.6f}, "
                f"current_lon={self.current_lon:.6f}, current_alt={self.current_alt:.2f}",
                ColorCodes.BLUE_OK,
            )"""
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
            """colorStr(
                f"New lat/lon goal received: lat={lat:.6f}, lon={lon:.6f} => (x={x:.2f}, y={y:.2f})",
                ColorCodes.BLUE_OK,
            )"""
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
        self.get_logger().info(f"Resolution: {msg.info.resolution}")
        self.global_costmap = msg
        self.end_goal_index = self.position_to_index(self.global_costmap, self.end_goal_waypoint)
        self.get_logger().info(
            f"occupancy grid has an origin of {self.global_costmap.info.origin.position.x},{self.global_costmap.info.origin.position.y}"
        )
        self.get_logger().info(f"resolution: {self.global_costmap.info.resolution}")
        # check first 10 cells
        print(msg.data[:10])

    # ----------------------
    #   Main Navigation Logic
    # ----------------------
    def updateNavigation(self) -> None:
        # If anchor not received, publish status but do not navigate
        if not self.anchor_received:
            self.get_logger().info("anchor not received, creating fake navigation data for testing")
            # self.publishStatus("No anchor received; Navigation Stopped.")
            # return
        # If no active waypoint
        if self.active_waypoint is None:
            # Plan a set of waypoints using a queue
            self.get_logger().info("No active waypoint, setting a default one for testing")

            if len(self.path.poses) > 0:
                path_length: int = max(len(self.path.poses) - 1, 0)
                self.active_waypoint = (
                    # self.path.poses[path_length].pose.position.x,
                    self.path.poses[0].pose.position.x,
                    # self.path.poses[path_length].pose.position.y,
                    self.path.poses[0].pose.position.y,
                )
            # self.publishStatus("No waypoint provided; Navigation Stopped.")
            # return

        # Compute distance to the waypoint
        goal_x, goal_y = self.active_waypoint
        self.get_logger().info(f"Goal X: {goal_x}, Goal Y: {goal_y}")
        dist_to_goal = self.distance_2d(
            self.current_position[0], self.current_position[1], goal_x, goal_y
        )

        pos_msg = Pose2D()
        pos_msg.x = self.current_position[0]
        pos_msg.y = self.current_position[1]
        pos_msg.theta = self.current_yaw
        self.pos_pub.publish(pos_msg)

        if dist_to_goal < self.reached_threshold:
            # Reached => Publish success, clear waypoint
            self.publishStatus(f"Successfully reached waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.active_waypoint = None
            return
        elif self.global_costmap != None:
            self.get_logger().warn("Running test")
            self.planPath(self.global_costmap)
            self.path_pub.publish(self.path)
            return
        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    def test_via_fire(self, grid: OccupancyGrid) -> None:
        # this is a test function that "fires" in a stright line on the occupancy grid
        # to use this function, throw a bunch of boxes in front of the camera, and esnure that it shows up on the costmap as blocked
        # if the translation functions are working, it should give you cost values that line up with the map
        length = 1
        starting_index = self.position_to_index(grid, (0.0, 0.0))  #
        self.get_logger().info(f"plotting point beginning at index  {starting_index} at 0,0")
        counter = int(length / grid.info.resolution)
        for i in range(0, counter):
            index = starting_index + (i)  # increasing the width takes you directly to the left
            x, y = self.index_to_position(grid, index)  #
            self.append_path((x, y))
            # going forwards by one row brings me to the left
            self.get_logger().info(
                f"index  {index} has a value of {grid.data[index]} and a position of {x},{y}"
            )

    def planPath(
        self,
        grid: OccupancyGrid,
    ) -> None:
        """
        This algorithm looks at the global occupancy grid in order to plan a path through it for the rover using an A* style search algorithm.
        """
        lowest_cost_position = self.current_position
        previous_position = lowest_cost_position
        distance_to_goal = self.distance_2d(
            lowest_cost_position[0],
            lowest_cost_position[1],
            self.end_goal_waypoint[0],
            self.end_goal_waypoint[1],
        )
        while distance_to_goal > 0.2:
            target_area = self.collect_adjacent(
                grid, self.position_to_index(grid, lowest_cost_position)
            )
            if len(target_area) == 0:
                # we have found no suitable indicies
                # pop the current position from the path
                self.path.poses.pop()
                # make the new lowest position the item at the top of the stack
                lowest_cost_position = self.path.poses.data[len(self.path.poses) - 1]
                # continue loop to avoid find lowest cost node
                continue

            lowest_cost_node = self.find_lowest_cost_node(target_area, grid)
            lowest_cost_position = self.index_to_position(grid, lowest_cost_node)
            self.append_path(lowest_cost_position)
            distance_to_goal = self.distance_2d(
                lowest_cost_position[0],
                lowest_cost_position[1],
                self.end_goal_waypoint[0],
                self.end_goal_waypoint[1],
            )
        self.get_logger().info(f"plotted to goal successfully")
        self.path_pub.publish(self.path)

    def find_lowest_cost_node(self, target_area: list[Tuple[int, int]], grid: OccupancyGrid) -> int:
        self.get_logger().info(f"target area is this large: {len(target_area)}")
        minimum_cost = sys.float_info.max
        for item in target_area:
            item_cost = self.distance_between_indicies(grid, item[1], self.end_goal_index)
            if item_cost < minimum_cost and item_cost != 100 and item_cost != -1:
                minimum_cost = item_cost
                minimum_index = item[1]

        return minimum_index

    def append_path(self, new_pose: Tuple[float, float]) -> None:

        self.path.header.frame_id = "map"
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = new_pose[0]
        pose.pose.position.y = new_pose[1]
        pose.pose.orientation.w = 1.0
        self.get_logger().info(f"adding position {new_pose[0]} x {new_pose[1]}")
        self.path.poses.append(pose)

    def distance_between_indicies(self, grid: OccupancyGrid, ind1: int, ind2: int) -> float:
        # distance = distance in sqrt((row_distance)^2+(collumn_distance)^2)
        row1 = ind1 // grid.info.width
        col1 = ind1 % grid.info.width
        row2 = ind2 // grid.info.width
        col2 = ind2 % grid.info.width
        distance = math.sqrt((row1 - row2) ** 2 + (col1 - col2) ** 2)
        return distance

    def collect_adjacent(self, grid: OccupancyGrid, current_index: int) -> list[Tuple[int, int]]:
        adjacent_points: list[Tuple[int, int]] = []
        grid.data[current_index] = 50
        if grid.data[current_index - 1] != 100 and grid.data[current_index - 1] != 50:
            adjacent_points.append((grid.data[current_index - 1], current_index - 1))
        if grid.data[current_index + 1] != 100 and grid.data[current_index + 1] != 50:
            adjacent_points.append((grid.data[current_index + 1], current_index + 1))
        if (
            grid.data[current_index - grid.info.width] != 100
            and grid.data[current_index - grid.info.width] != 50
        ):
            adjacent_points.append(
                (grid.data[current_index - grid.info.width], current_index - grid.info.width)
            )
        if (
            grid.data[current_index + grid.info.width] != 100
            and grid.data[current_index + grid.info.width] != 50
        ):
            adjacent_points.append(
                (grid.data[current_index + grid.info.width], current_index + grid.info.width)
            )
        return adjacent_points

    def position_to_index(self, grid: OccupancyGrid, position: Tuple[float, float]) -> int:
        # this function takes in the occupancy grid and an index within in it, and returns the map coordinates of that point
        # for testing, i have swapped the row and column variables (10/28/2025)
        x, y = position
        col = int(
            (x - grid.info.origin.position.x) / grid.info.resolution
        )  # the number of cells forward
        row = int(
            (y - grid.info.origin.position.y) / grid.info.resolution
        )  # the number of cells side to side
        current_index = int((row * grid.info.width) + col)
        if current_index > len(grid.data):
            self.get_logger().warn(f"the position  {x}, {y} is outside the occupancy grid")
        return current_index

    def index_to_position(self, grid: OccupancyGrid, target_index: int) -> Tuple[float, float]:
        x_grid = target_index % grid.info.width  # the number of rows we have gone up or down
        y_grid = target_index // grid.info.width
        x_position = grid.info.origin.position.x + (x_grid + 0.5) * grid.info.resolution
        y_position = grid.info.origin.position.y + (y_grid + 0.5) * grid.info.resolution
        # self.get_logger().info(f"index {target_index} has a position of {x_position},{y_position}")
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
