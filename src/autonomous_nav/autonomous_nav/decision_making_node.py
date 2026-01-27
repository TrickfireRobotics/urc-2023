"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav2_simple_commander.costmap_2d import PyCostmap2D

# from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Float32, String
from tf_transformations import euler_from_quaternion

# from .dwa_planner import DWAPlanner

# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


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

        # Add Nav2 action client
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")

        # Navigator client
        # self.nav_client = BasicNavigator()

        # Velocity tracking - initialize velocity to 0.5 to get rover moving
        self.current_wheel_vel = (0.5, 0.5)
        # self.current_wheel_vel = (0.0, 0.0)
        self.last_left_vel = 0.0
        self.last_right_vel = 0.0

        # Waypoint list (stores waypoints in global frame)  - Updated to Path message
        # self.waypoint_list: List[Tuple[float, float]] = []
        self.waypoint_path: Path = Path()
        self.waypoint_reached_threshold = 0.05  # meters

        # Nav2 action tracking
        self.goal_handle: ClientGoalHandle[FollowPath] = None
        self.is_navigating = False
        self.navigation_complete = False

        # Robot physical parameters - rough estimates from rover
        self.wheel_base = 0.5
        self.wheel_radius = 0.11

        # SPEED = 6.28 * 2.5 = 15.7 rad/s (from drivebase.py)
        self.drivebase_speed_multiplier = 6.28 * 2.5

        # Costmap with no data.
        # self.costmap: Optional[PyCostmap2D] = None
        # self.occupancy_grid: PyCostmap2D = PyCostmap2D(OccupancyGrid())
        self.costmap: PyCostmap2D = PyCostmap2D(OccupancyGrid())

        # # DWA Planner
        # self.dwa_planner: DWAPlanner = DWAPlanner(
        #     costmap=self.costmap,
        #     robot_radius=0.3,
        #     current_velocity=self.current_wheel_vel,
        #     current_position=(0.0, 0.0),
        #     time_delta=0.1,
        #     goal=(0, 0),
        #     theta=self.global_theta,
        # )

        # Navigation status
        self.navigation_status = "No waypoint provided"

        # ===== SUBSCRIBERS =====

        # Subscribe to Nav2's velocity output
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

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

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """Update costmap and initialize planner if needed."""
        self.costmap = PyCostmap2D(msg)
        self.get_logger().info(
            f"Occupancygrid updated with {msg.info.width} x {msg.info.height} grid"
        )

    # def path_callback(self, msg: Path) -> None:
    #     """Receive new waypoint queue."""
    #     self.get_logger().info("Received new path")

    #     self.waypoint_list.clear()

    #     # Take first 10 waypoints
    #     for pose in msg.poses:
    #         x = pose.pose.position.x
    #         y = pose.pose.position.y
    #         self.waypoint_list.append((x, y))

    #     self.get_logger().info(f"Received path with {len(self.waypoint_list)} waypoints")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Convert Nav2 Twist commands to drivebase format."""
        linear_vel = msg.linear.x  # m/s
        angular_vel = msg.angular.z  # rad/s

        # Differential drive kinematics
        left_linear = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_linear = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to wheel angular velocity (rad/s)
        left_wheel = left_linear / self.wheel_radius
        right_wheel = right_linear / self.wheel_radius

        # Normalize for drivebase
        left_normalized = left_wheel / self.drivebase_speed_multiplier
        right_normalized = right_wheel / self.drivebase_speed_multiplier

        # Clamp and publish
        left_clamped = max(-1.0, min(1.0, left_normalized))
        right_clamped = max(-1.0, min(1.0, right_normalized))

        self.left_drive_pub.publish(Float32(data=left_clamped))
        self.right_drive_pub.publish(Float32(data=right_clamped))

    def path_callback(self, msg: Path) -> None:
        """Send path to Nav2 controller."""
        self.get_logger().info(f"Received path with {len(msg.poses)} poses")

        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path")
            return

        # Store the path
        self.waypoint_path = msg

        # Wait for action server
        if not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("FollowPath action server not available")
            return

        # Cancel any existing goal
        if self.goal_handle is not None and self.is_navigating:
            self.get_logger().info("Canceling previous goal")
            self.goal_handle.cancel_goal_async()

        # Send goal to Nav2 controller
        goal = FollowPath.Goal()
        goal.path = msg

        self.get_logger().info("Sending path to Nav2 controller")
        send_goal_future = self.follow_path_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def nav_status_callback(self, msg: String) -> None:
        """Update navigation status."""
        self.get_logger().info(f"Navigation Status: {msg.data}")
        self.navigation_status = msg.data

    # ===== NAV2 ACTION CALLBACKS =====

    def goal_response_callback(self, future: Future[ClientGoalHandle[FollowPath]]) -> None:
        """Handle Nav2 action server's response to our goal request."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected by Nav2")
            self.is_navigating = False
            return

        self.get_logger().info("FollowPath goal accepted by Nav2")
        self.is_navigating = True
        self.navigation_complete = False

        # Get notified when navigation completes
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg: FollowPath.Impl.FeedbackMessage) -> None:
        """Receive progress updates from Nav2 controller."""
        feedback: FollowPath.Feedback = feedback_msg.feedback
        # FollowPath feedback contains distance_to_goal and speed
        self.get_logger().info(
            f"Nav2 feedback - Distance remaining: {feedback.distance_to_goal:.2f}m, "
            f"Speed: {feedback.speed:.2f}m/s"
        )

    def result_callback(
        self, future: Future[ClientGoalHandle[FollowPath].GetResultResponse]
    ) -> None:
        """Handle navigation completion or failure."""
        # result: ClientGoalHandle[FollowPath].GetResultResponse = future.result()
        result = future.result()
        status: int = result.status

        self.is_navigating = False
        self.navigation_complete = True

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation completed successfully!")
            self.waypoint_path = Path()  # Clear the path
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation was canceled")
        else:  # ABORTED or other failure
            self.get_logger().warn(f"Navigation failed with status: {status}")

        self.stop_rover()

    # ===== MAIN LOGIC =====

    # def update_decision(self) -> None:
    #     """Main control loop - runs at 2 Hz."""

    #     self.get_logger().info("Updating decision making...")

    #     # Check if we have a valid costmap (non-empty)
    #     if self.costmap.getSizeInCellsX() == 0 or self.costmap.getSizeInCellsY() == 0:
    #         self.get_logger().info("No costmap received yet, waiting...")
    #         self.stop_rover()
    #         return

    #     # Check if we have waypoints
    #     if not self.waypoint_list:
    #         self.stop_rover()
    #         return

    #     # Get current waypoint
    #     if len(self.waypoint_list) < 1:
    #         self.get_logger().info("No waypoints to navigate to.")
    #         self.stop_rover()
    #         return
    #     current_goal_global = self.waypoint_list[0]

    #     current_goal_global = (
    #         current_goal_global[0],
    #         current_goal_global[1],
    #     )

    #     self.get_logger().info(
    #         f"Navigating to waypoint ({current_goal_global[0]}, {current_goal_global[1]})"
    #     )

    #     # Check if reached current waypoint
    #     distance_to_goal = math.sqrt(
    #         (current_goal_global[0] - self.global_x) ** 2
    #         + (current_goal_global[1] - self.global_y) ** 2
    #     )

    #     if distance_to_goal < self.waypoint_reached_threshold:
    #         self.waypoint_list.pop(0)
    #         self.get_logger().info(f"Reached waypoint!")
    #         if not self.waypoint_list:
    #             self.stop_rover()
    #             return

    #         # Update to next waypoint
    #         current_goal_global = self.waypoint_list[0]

    #     if self.costmap is not None:
    #         self.local_x = self.costmap.getSizeInCellsX()

    #     # Transform goal from global (odom) to local (robot/costmap frame)
    #     goal_local = self.transform_global_to_local(current_goal_global)

    #     # Update DWA planner state
    #     self.get_logger().info("Updating states")
    #     self.dwa_planner.update_state(
    #         costmap=self.costmap,
    #         current_position=(0.0, 0.0),
    #         current_theta=self.global_theta,
    #         current_velocity=self.current_wheel_vel,
    #         goal=goal_local,
    #         global_pose=(self.global_x, self.global_y, self.global_theta),
    #     )

    #     # Plan and execute

    #     # debugging statements
    #     left_vel: Float32 = 1.0
    #     right_vel: Float32 = 1.0
    #     velocities: Tuple[float, float] = self.dwa_planner.plan()
    #     self.get_logger().info(f" Velocities: {velocities}")

    #     self.get_logger().info("Getting wheel velocities")
    #     # left_vel, right_vel = self.dwa_planner.plan()
    #     self.get_logger().info(f"Left Vel: {left_vel:.2f}, Right Vel: {right_vel:.2f}")

    #     # Store for next cycle
    #     self.last_left_vel = left_vel
    #     self.last_right_vel = right_vel

    #     self.publish_drive_commands(left_vel, right_vel)

    def update_decision(self) -> None:
        """Main control loop - monitors Nav2 status at 2 Hz."""

        # Check if Nav2 is actively navigating
        if not self.is_navigating:
            if not self.waypoint_path.poses:
                # No path and not navigating - nothing to do
                return
            else:
                # Have a path but not navigating - might need to resend
                self.get_logger().debug("Have path but not navigating")
            return

        # Log current status while navigating
        if len(self.waypoint_path.poses) > 0:
            final_pose = self.waypoint_path.poses[-1].pose.position
            distance_to_goal = math.sqrt(
                (final_pose.x - self.global_x) ** 2 + (final_pose.y - self.global_y) ** 2
            )
            self.get_logger().info(f"Navigating - Distance to final goal: {distance_to_goal:.2f}m")

            # Manual goal check (backup to Nav2's internal check)
            if distance_to_goal < self.waypoint_reached_threshold:
                self.get_logger().info("Reached final waypoint!")
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()

        # If not navigating, then stop rover

        # Handle not navigating but has path
        if not self.is_navigating and self.waypoint_path.poses:
            self.get_logger().info("Rover not navigating, but has Path")

        # Handle timeout/stuck detection

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
        self.get_logger().info("Stopping rover")
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
