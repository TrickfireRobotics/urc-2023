import math
import sys
from typing import Optional, Tuple

import rclpy
import rclpy.future
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String


class NavigationNode(Node):
    """
    A ROS 2 node for handling robot navigation using latitude/longitudes waypoints
    relative to a dynamically received anchor lat/lon. This node manages the navigation status
    (are we there yet?) and plots an initial path to our goal coordinate.

    This node:
        - Subscribes to /anchor_position (Float64MultiArray).
        - Subscribes to /goal_latlon (NavSatFix) for a new lat/lon waypoint.
        - Subscribes to /odometry/filtered (Odometry) for current pose.
        - Publishes /navigation_status (String) and /navigation_feedback (Pose2D).
        - Publishes /path (Path) to be followed by a lower level node.

    Global Path Planning:
        - use_nav2 = True  => uses Nav2's ComputePathToPose action (NavFn planner)
        - use_nav2 = False => uses the original custom A* search algorithm
        Toggle this flag to compare performance between the two planners.
    """

    def __init__(self) -> None:
        super().__init__("navigation_node")
        self.get_logger().info("Navigation Node has been started successfully...")

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
        self.index_count: int = 0
        self.get_logger().info("Navigation Node has initialized first arguments")

        # ---- Global Path Planner Toggle ----
        # Set use_nav2 = True to use Nav2's NavFn planner (new)
        # Set use_nav2 = False to use the original custom A* planner
        self.use_nav2: bool = True
        self._nav2_path_pending: bool = False  # prevents duplicate requests

        # ---- Subscribers ----
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

        # ---- Nav2 Global Planner Action Client ----
        self._nav2_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        # ---- Timers ----
        self.timer = self.create_timer(5, self.updateNavigation)  # 0.2 Hz

    # ----------------------
    #   Subscriptions
    # ----------------------
    def anchorCallback(self, msg: Float64MultiArray) -> None:
        """Receives the anchor position [lat, lon, alt]."""
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
                f"Anchor received. ref_lat={self.ref_lat:.6f}, "
                f"ref_lon={self.ref_lon:.6f}, ref_alt={self.ref_alt:.2f}"
            )

    def gpsCallback(self, msg: NavSatFix) -> None:
        if not self.anchor_received:
            return
        if msg.status.status >= 0 and self.current_lat == 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_alt = msg.altitude
            self.determine_global_yaw()
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_alt = msg.altitude
        self.get_logger().info(
            f"current position received. current_lat={self.current_lat:.6f}, "
            f"current_lon={self.current_lon:.6f}, current_alt={self.current_alt:.2f}"
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
        """Converts lat/lon from the NavSatFix message into local x,y coordinates."""
        if not self.anchor_received:
            self.get_logger().warn("Received /goal_latlon but anchor not set yet.")
            return
        lat = msg.latitude
        lon = msg.longitude
        x, y = self.convertLatLonToXY(lat, lon)
        # TODO these two variables track the same thing, fix it
        self.active_waypoint = (x, y)
        self.end_goal_waypoint = (x, y)
        # Reset path so a new one gets planned to the new goal
        self.path = Path()
        self._nav2_path_pending = False
        self.get_logger().info(
            f"New lat/lon goal received: lat={lat:.6f}, lon={lon:.6f} => (x={x:.2f}, y={y:.2f})"
        )

    def odomCallback(self, msg: Odometry) -> None:
        """Updates self.current_position and self.current_yaw from odometry."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def costmap_callback(self, msg: OccupancyGrid) -> None:
        self.get_logger().info(f"Received costmap: {msg.info.width} x {msg.info.height}")
        self.get_logger().info(f"Resolution: {msg.info.resolution}")
        self.global_costmap = msg
        self.end_goal_index = self.position_to_index(self.global_costmap, self.end_goal_waypoint)
        self.get_logger().info(
            f"occupancy grid has an origin of "
            f"{self.global_costmap.info.origin.position.x},"
            f"{self.global_costmap.info.origin.position.y}"
        )
        print(msg.data[:10])

    # ----------------------
    #   Main Navigation Logic
    # ----------------------
    def updateNavigation(self) -> None:
        if not self.anchor_received:
            self.get_logger().info(
                "anchor not received, creating sample navigation data for testing"
            )

        if self.active_waypoint is None:
            self.get_logger().info("No active waypoint, setting a default one for testing")
            if len(self.path.poses) > 0:
                self.active_waypoint = (
                    self.path.poses[0].pose.position.x,
                    self.path.poses[0].pose.position.y,
                )
                return

        if self.active_waypoint is not None:
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
            self.publishStatus(f"Successfully reached waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.active_waypoint = None
            return
        elif len(self.path.poses) == 0 and not self._nav2_path_pending:
            if self.use_nav2:
                self.get_logger().info("Requesting path from Nav2 global planner...")
                self.planPathNav2()
            elif self.global_costmap is not None:
                self.get_logger().warn("Running custom A* planner...")
                self.planPathCustom(self.global_costmap)
                self.get_logger().info(f"Custom A* path plotted. Total indices queried: {self.index_count}")
                self.path_pub.publish(self.path)
            return

        self.publishStatus(f"En route to waypoint ({goal_x:.2f}, {goal_y:.2f})")
        self.publishFeedback(goal_x, goal_y)

    # ----------------------
    #   Nav2 Global Planner (NEW)
    # ----------------------
    def planPathNav2(self) -> None:
        """
        Plans a path using Nav2's ComputePathToPose action server (NavFn planner).
        This is an async call -- the result comes back via _nav2_result_callback.
        Toggle self.use_nav2 = False to use the original custom A* instead.
        """
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                "Nav2 planner server not available. Falling back to custom A*."
            )
            self.use_nav2 = False
            if self.global_costmap is not None:
                self.planPathCustom(self.global_costmap)
                self.path_pub.publish(self.path)
            return

        goal_msg = ComputePathToPose.Goal()

        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.current_position[0]
        start.pose.position.y = self.current_position[1]
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.end_goal_waypoint[0]
        goal.pose.position.y = self.end_goal_waypoint[1]
        goal.pose.orientation.w = 1.0

        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = "NavFn"

        self._nav2_path_pending = True
        self.get_logger().info("Sending path request to Nav2 planner (NavFn)...")
        send_goal_future = self._nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav2_goal_response_callback)

    def _nav2_goal_response_callback(self, future: rclpy.Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 planner rejected the goal.")
            self._nav2_path_pending = False
            return
        self.get_logger().info("Nav2 planner accepted goal, waiting for path result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_callback)

    def _nav2_result_callback(self, future: rclpy.Future) -> None:
        result = future.result().result
        self.path = result.path
        self._nav2_path_pending = False
        self.get_logger().info(
            f"Nav2 planner returned path with {len(self.path.poses)} poses."
        )
        self.path_pub.publish(self.path)

    # ----------------------
    #   Original Custom A* Planner (kept for comparison)
    # ----------------------
    def planPathCustom(self, grid: OccupancyGrid) -> None:
        """
        Original custom A*-style greedy search planner.
        Kept for performance comparison against Nav2's NavFn planner.
        Set self.use_nav2 = False to use this instead.
        """
        publish_count = 0
        lowest_cost_position = self.current_position
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
                self.get_logger().info("backtracking")
                if len(self.path.poses) != 0:
                    self.path.poses.pop()
                lowest_cost_position = (
                    self.path.poses[-1].pose.position.x,
                    self.path.poses[-1].pose.position.y,
                )
                continue

            lowest_cost_node = self.find_lowest_cost_node(target_area, grid)
            lowest_cost_position = self.index_to_position(grid, lowest_cost_node)
            if publish_count % 10 == 0:
                self.get_logger().info(
                    f"current lowest cost position is {lowest_cost_position[0]},{lowest_cost_position[1]}"
                )
                self.get_logger().info(f"current path length is {publish_count}")
                self.append_path(lowest_cost_position)
            publish_count += 1
            distance_to_goal = self.distance_2d(
                lowest_cost_position[0],
                lowest_cost_position[1],
                self.end_goal_waypoint[0],
                self.end_goal_waypoint[1],
            )

    def find_lowest_cost_node(
        self, target_area: list[Tuple[int, int]], grid: OccupancyGrid
    ) -> int:
        minimum_cost = sys.float_info.max
        minimum_index = None
        if target_area == []:
            self.get_logger().warn("target area is empty, cannot find lowest cost node")
            return self.position_to_index(grid, self.current_position)
        for item in target_area:
            item_value = item[0]
            item_index = item[1]
            item_cost = self.distance_between_indicies(grid, item_index, self.end_goal_index)
            if item_cost < minimum_cost and item_value != 100 and item_value != 50:
                minimum_cost = item_cost
                minimum_index = item_index
        if minimum_index is None:
            self.get_logger().warn("no valid nodes found in target area, defaulting to current position")
            return self.position_to_index(grid, self.current_position)
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

    def distance_between_indicies(
        self, grid: OccupancyGrid, ind1: int, ind2: int
    ) -> float:
        row1 = ind1 // grid.info.width
        col1 = ind1 % grid.info.width
        row2 = ind2 // grid.info.width
        col2 = ind2 % grid.info.width
        return math.sqrt((row1 - row2) ** 2 + (col1 - col2) ** 2)

    def collect_adjacent(
        self, grid: OccupancyGrid, current_index: int
    ) -> list[Tuple[int, int]]:
        adjacent_points: list[Tuple[int, int]] = []
        grid.data[current_index] = 50
        self.index_count += 4
        if grid.data[current_index - 1] != 100 and grid.data[current_index - 1] != 50:
            adjacent_points.append((grid.data[current_index - 1], current_index - 1))
        if grid.data[current_index + 1] != 100 and grid.data[current_index + 1] != 50:
            adjacent_points.append((grid.data[current_index + 1], current_index + 1))
        if (
            grid.data[current_index - grid.info.width] != 100
            and grid.data[current_index - grid.info.width] != 50
        ):
            adjacent_points.append(
                (
                    grid.data[current_index - grid.info.width],
                    current_index - grid.info.width,
                )
            )
        if (
            grid.data[current_index + grid.info.width] != 100
            and grid.data[current_index + grid.info.width] != 50
        ):
            adjacent_points.append(
                (
                    grid.data[current_index + grid.info.width],
                    current_index + grid.info.width,
                )
            )
        return adjacent_points

    def position_to_index(
        self, grid: OccupancyGrid, position: Tuple[float, float]
    ) -> int:
        x, y = position
        col = int((x - grid.info.origin.position.x) / grid.info.resolution)
        row = int((y - grid.info.origin.position.y) / grid.info.resolution)
        current_index = int((row * grid.info.width) + col)
        if current_index > len(grid.data):
            self.get_logger().warn(f"the position {x}, {y} is outside the occupancy grid")
        return current_index

    def index_to_position(
        self, grid: OccupancyGrid, target_index: int
    ) -> Tuple[float, float]:
        x_grid = target_index % grid.info.width
        y_grid = target_index // grid.info.width
        x_position = grid.info.origin.position.x + (x_grid + 0.5) * grid.info.resolution
        y_position = grid.info.origin.position.y + (y_grid + 0.5) * grid.info.resolution
        return (x_position, y_position)

    def test_via_fire(self, grid: OccupancyGrid) -> None:
        length = 1
        starting_index = self.position_to_index(grid, (0.0, 0.0))
        self.get_logger().info(f"plotting point beginning at index {starting_index} at 0,0")
        counter = int(length / grid.info.resolution)
        for i in range(0, counter):
            index = starting_index + i
            x, y = self.index_to_position(grid, index)
            self.get_logger().info(
                f"index {index} has a value of {grid.data[index]} and a position of {x},{y}"
            )

    def turnTowardGoal(self, goal_Location: Tuple[float, float]) -> None:
        a = self.distance_2d(
            self.current_position[0],
            self.current_position[1],
            goal_Location[0],
            goal_Location[1],
        )
        b = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        c = self.distance_2d(goal_Location[0], goal_Location[1], self.start_lat, self.start_lon)
        temp = (a**2 - c**2 - b**2) / (-2 * b * c)
        turn_angle = math.degrees(math.acos(temp))

    # ----------------------
    #   Publishing Helpers
    # ----------------------
    def publishStatus(self, msg: str) -> None:
        self.status_pub.publish(String(data=msg))

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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()