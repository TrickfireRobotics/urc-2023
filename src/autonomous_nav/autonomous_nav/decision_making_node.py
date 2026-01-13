"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.
"""

import math
from typing import List, Tuple

import rclpy
from nav2_simple_commander.costmap_2d import PyCostmap2D

# from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String

# from transforms3d.euler import quat2euler
from tf_transformations import euler_from_quaternion

from .dwa_planner import DWAPlanner

# from lib.color_codes import ColorCodes, colorStr


class DecisionMakingNode(Node):
    def __init__(self) -> None:
        super().__init__("decision_making_node")

        # ===== STATE VARIABLES =====

        # Global pose tracking (from odometry)
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_theta = 0.0

        # Local/costmap frame tracking
        self.local_x = 0.0
        self.local_y = 0.0

        # Velocity tracking - initialize velocity to 0.5 to get rover moving
        self.current_wheel_vel = (0.5, 0.5)
        # self.current_wheel_vel = (0.0, 0.0)
        self.last_left_vel = 0.0
        self.last_right_vel = 0.0

        # Waypoint list (stores waypoints in global frame)  - Updated to Path message
        self.waypoint_list: List[Tuple[float, float]] = []
        self.waypoint_reached_threshold = 0.05  # meters

        # Costmap with no data.
        # self.costmap: Optional[PyCostmap2D] = None
        # self.occupancy_grid: PyCostmap2D = PyCostmap2D(OccupancyGrid())
        self.costmap: PyCostmap2D = PyCostmap2D(OccupancyGrid())

        # DWA Planner
        self.dwa_planner: DWAPlanner = DWAPlanner(
            costmap=self.costmap,
            robot_radius=0.3,
            current_velocity=self.current_wheel_vel,
            current_position=(0.0, 0.0),
            time_delta=0.1,
            goal=self.waypoint_list[0],
            theta=self.global_theta,
        )

        # Navigation status
        self.navigation_status = "No waypoint provided"

        # ===== SUBSCRIBERS =====

        # Odometry for global pose tracking
        self.create_subscription(Odometry, "/odometry/filtered", self.odometry_callback, 10)

        # Costmap (rolling window)
        self.create_subscription(OccupancyGrid, "/projected_map", self.occupancy_grid_callback, 10)

        # Waypoint path
        self.create_subscription(Path, "/path", self.path_callback, 10)

        # Navigation status
        self.create_subscription(String, "/navigation_status", self.nav_status_callback, 10)

        # ===== PUBLISHERS =====

        # Publishers to be used if the control node is used
        # self.left_drive_pub = self.create_publisher(Float32, "/left_wheel_velocity", 10)
        # self.right_drive_pub = self.create_publisher(Float32, "/right_wheel_velocity", 10)

        self.left_drive_pub = self.create_publisher(
            Float32, "/move_left_drivebase_side_message", 10
        )
        self.right_drive_pub = self.create_publisher(
            Float32, "/move_right_drivebase_side_message", 10
        )

        # ===== TIMER =====
        self.timer = self.create_timer(0.5, self.update_decision)  # 2 Hz

        self.get_logger().info("DecisionMakingNode initialized")

    # ===== CALLBACKS =====

    def odometry_callback(self, msg: Odometry) -> None:
        self.global_x = msg.pose.pose.position.x
        self.global_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation

        _, _, self.global_theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z

        wheel_base = 0.5
        wheel_radius = 0.11

        left_linear = linear_vel - (angular_vel * wheel_base / 2.0)
        right_linear = linear_vel + (angular_vel * wheel_base / 2.0)

        self.current_wheel_vel = (left_linear / wheel_radius, right_linear / wheel_radius)

    # def odometry_callback(self, msg: Odometry) -> None:
    #     """Update global pose from odometry."""
    #     self.get_logger().info("Received odometry update")
    #     self.global_x = msg.pose.pose.position.x
    #     self.global_y = msg.pose.pose.position.y

    #     # Extract yaw from quaternion
    #     orientation = msg.pose.pose.orientation
    #     _, _, self.global_theta = quat2euler(
    #         [orientation.w, orientation.x, orientation.y, orientation.z], "sxyz"
    #     )

    #     # Extract wheel velocities from twist (if available)
    #     # Or estimate from linear/angular velocity
    #     linear_vel = msg.twist.twist.linear.x
    #     angular_vel = msg.twist.twist.angular.z

    #     # Convert to wheel velocities (inverse kinematics)
    #     wheel_base = 0.5  # Match DWA planner
    #     wheel_radius = 0.11

    #     left_linear = linear_vel - (angular_vel * wheel_base / 2.0)
    #     right_linear = linear_vel + (angular_vel * wheel_base / 2.0)

    #     self.current_wheel_vel = (left_linear / wheel_radius, right_linear / wheel_radius)

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """Update costmap and initialize planner if needed."""
        self.costmap = PyCostmap2D(msg)
        self.get_logger().info(
            f"Occupancygrid updated with {msg.info.width} x {msg.info.height} grid"
        )

    def path_callback(self, msg: Path) -> None:
        """Receive new waypoint queue."""
        self.get_logger().info("Received new path")

        self.waypoint_list.clear()

        # Take first 10 waypoints
        for i, pose_stamped in enumerate(msg.poses):
            pose = pose_stamped.pose
            x = pose_stamped.position.x
            y = pose_stamped.position.y
            self.waypoint_list.append((x, y))

        self.get_logger().info(f"Received path with {len(self.waypoint_list)} waypoints")

    def nav_status_callback(self, msg: String) -> None:
        """Update navigation status."""
        self.get_logger().info(f"Navigation Status: {msg.data}")
        self.navigation_status = msg.data

    # ===== MAIN LOGIC =====

    def update_decision(self) -> None:
        """Main control loop - runs at 2 Hz."""

        self.get_logger().info("Updating decision making...")

        # Check if we have a valid costmap (non-empty)
        if self.costmap.getSizeInCellsX() == 0 or self.costmap.getSizeInCellsY() == 0:
            self.get_logger().info("No costmap to navigate through")
            # self.stop_rover()
            # return

            # Create a PyCostmap2D from OccupancyGrid
            self.costmap = PyCostmap2D(self.costmap)
            # # Create fake costmap for testing
            # self.get_logger().info("Costmap not initialized or empty, creating fake costmap for testing")
            # fake_grid.info.resolution = 0.1
            # fake_grid = OccupancyGrid()
            # fake_grid.info.width = 100
            # fake_grid.info.height = 100
            # fake_grid.info.origin.position.x = self.global_x - 5.0
            # fake_grid.info.origin.position.y = self.global_y - 5.0
            # fake_grid.data = [0] * (100 * 100)  # Initialize with free space
            # self.costmap = PyCostmap2D(fake_grid)  # Actually assign the fake costmap

        # Check if we have waypoints
        if not self.waypoint_list:
            self.stop_rover()
            return

        # Get current waypoint
        if len(self.waypoint_list) < 1:
            self.get_logger().info("There are no waypoints to navigate to.")
            self.stop_rover()
            return
        current_goal_global = self.waypoint_list[0]

        current_goal_global = (
            current_goal_global[0],
            current_goal_global[1],
        )

        self.get_logger().info(
            f"Navigating to waypoint ({current_goal_global[0]}, {current_goal_global[1]})"
        )

        # Check if reached current waypoint
        distance_to_goal = math.sqrt(
            (current_goal_global[0] - self.global_x) ** 2
            + (current_goal_global[1] - self.global_y) ** 2
        )

        if distance_to_goal < self.waypoint_reached_threshold:
            self.waypoint_list.pop(0)
            self.get_logger().info(f"Reached waypoint!")
            if not self.waypoint_list:
                self.stop_rover()
                return

            # Update to next waypoint
            current_goal_global = self.waypoint_list[0]

        if self.costmap is not None:
            self.local_x = self.costmap.getSizeInCellsX()

        # Transform goal from global (odom) to local (robot/costmap frame)
        goal_local = self.transform_global_to_local(current_goal_global)

        # Update DWA planner state
        self.get_logger().info("Updating states")
        self.dwa_planner.update_state(
            costmap=self.costmap,
            current_position=(0.0, 0.0),
            current_theta=self.global_theta,
            current_velocity=self.current_wheel_vel,
            goal=goal_local,
            global_pose=(self.global_x, self.global_y, self.global_theta),
        )

        # Plan and execute
        self.get_logger().info("Getting wheel velocities")
        left_vel, right_vel = self.dwa_planner.plan()
        self.get_logger().info(f"Left Vel: {left_vel:.2f}, Right Vel: {right_vel:.2f}")

        # Store for next cycle
        self.last_left_vel = left_vel
        self.last_right_vel = right_vel

        self.publish_drive_commands(left_vel, right_vel)

    # ===== HELPER FUNCTIONS =====

    def transform_global_to_local(self, global_point: Tuple[float, float]) -> Tuple[float, float]:
        """
        Transform a point from global (odom) frame to local (robot) frame.

        For rolling window costmap centered on robot:
        - Robot is at (0, 0) in costmap frame
        - Need to rotate and translate global point to robot's perspective
        """
        # Vector from robot to goal in global frame
        dx_global = global_point[0] - self.global_x
        dy_global = global_point[1] - self.global_y

        # Rotate to robot's local frame
        cos_theta = math.cos(-self.global_theta)
        sin_theta = math.sin(-self.global_theta)

        local_x = dx_global * cos_theta - dy_global * sin_theta
        local_y = dx_global * sin_theta + dy_global * cos_theta

        return (local_x, local_y)

    def transform_path_to_list(self) -> List[Tuple[float, float]]:
        """Convert Path message to waypoint queue."""
        self.waypoint_list.clear()
        path_list: List[Tuple[float, float]] = []
        for pose_stamped in self.waypoint_list:
            x = pose_stamped[0]
            y = pose_stamped[1]
            path_list.append((x, y))
        return path_list

    def stop_rover(self) -> None:
        """Publish zero velocity."""
        # self.get_logger.info("Stopping rover")
        self.publish_drive_commands(0.0, 0.0)

    def publish_drive_commands(self, left_speed: float, right_speed: float) -> None:
        """Publish motor commands."""
        self.get_logger().info(f"Left Vel: {left_speed:.2f}, Right Vel: {right_speed:.2f}")
        self.left_drive_pub.publish(Float32(data=left_speed))
        self.right_drive_pub.publish(Float32(data=right_speed))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    decision_making_node = None
    try:
        decision_making_node = DecisionMakingNode()
        rclpy.spin(decision_making_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if decision_making_node is not None:
            # decision_making_node.get_logger().info(
            #     colorStr("Shutting down decision_making_node", ColorCodes.BLUE_OK)
            # )
            decision_making_node.get_logger().info("Shutting down decision_making_node")
    finally:
        if decision_making_node is not None:
            decision_making_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
