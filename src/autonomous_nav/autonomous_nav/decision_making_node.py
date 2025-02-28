"""
Role: This script defines the autonomous node that acts as the high-level controller.

Functionality:
    - Receives inputs from Navigation (/navigation_feedback, /navigation_status),
      Sensor Processing (/obstacle_detected, /obstacle_info), and (optionally) Localization
      to make decisions on how to drive the rover to its waypoint.
    - Publishes commands to the drivebase to move or stop.
"""

import sys
import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Bool, Float32, Float32MultiArray

from lib.color_codes import ColorCodes, colorStr


class DecisionMakingNode(Node):
    """
    A ROS 2 node that handles decision-making for a robot.

    Subscriptions:
      - /navigation_feedback (Pose2D): (dx, dy, dtheta) from the NavigationNode.
      - /navigation_status (String): e.g. "En route...", "No waypoint...", "Reached waypoint"
      - /obstacle_detected (Bool): True if a large obstacle is present.
      - /obstacle_info (Float32MultiArray): Additional obstacle data (e.g. average distance).

    Publications:
      - /move_left_drivebase_side_message (Float32)
      - /move_right_drivebase_side_message (Float32)

    Basic finite-state logic:
      1. If no waypoint or it's reached, stop.
      2. If obstacle is present, pivot left until clear, then drive forward for 3s
         to ensure we bypass the obstacle.
      3. Otherwise, steer toward the waypoint using heading error.
    """

    def __init__(self) -> None:
        super().__init__("decision_making_node")

        # ---- State Variables ----
        self.obstacle_detected: bool = False
        self.obstacle_info: list[float] = []  # Mypy: typed list of floats

        self.navigation_status: str = "No waypoint provided; Navigation Stopped."
        self.nav_feedback = Pose2D()

        # Finite-state variable for obstacle avoidance
        # Mypy fix: allow str or None
        self.avoid_state: Optional[str] = None
        self.avoid_start_time = self.get_clock().now()

        # ---- Subscribers ----
        self.create_subscription(Bool, "/obstacle_detected", self.obstacleCallback, 10)
        self.create_subscription(Float32MultiArray, "/obstacle_info", self.obstacleInfoCallback, 10)
        self.create_subscription(String, "/navigation_status", self.navStatusCallback, 10)
        self.create_subscription(Pose2D, "/navigation_feedback", self.navFeedbackCallback, 10)

        # ---- Publishers (to Drivebase) ----
        self.left_drive_pub = self.create_publisher(Float32, "move_left_drivebase_side_message", 10)
        self.right_drive_pub = self.create_publisher(
            Float32, "move_right_drivebase_side_message", 10
        )

        # Timer to run decision logic at ~10Hz
        self.timer = self.create_timer(0.1, self.updateDecision)

        self.get_logger().info(colorStr("DecisionMakingNode started.", ColorCodes.BLUE_OK))

    # --------------------------------------------------------------------------
    #   Subscription Callbacks
    # --------------------------------------------------------------------------
    def obstacleCallback(self, msg: Bool) -> None:
        self.obstacle_detected = msg.data

    def obstacleInfoCallback(self, msg: Float32MultiArray) -> None:
        self.obstacle_info = list(msg.data)

    def navStatusCallback(self, msg: String) -> None:
        self.navigation_status = msg.data
        # self.get_logger().info(
        #     colorStr(f"Navigation Status: {self.navigation_status}", ColorCodes.YELLOW_WARN)
        # )

    def navFeedbackCallback(self, msg: Pose2D) -> None:
        self.nav_feedback = msg

    # --------------------------------------------------------------------------
    #   Main Decision Logic
    # --------------------------------------------------------------------------
    def updateDecision(self) -> None:
        """
        Periodically checks obstacles, waypoint status, and decides how to drive.
        """
        # If no waypoint or reached
        if (
            "No waypoint provided" in self.navigation_status
            or "Successfully reached" in self.navigation_status
        ):
            self.stopRover()
            self.avoid_state = None  # Reset
            return

        # Obstacle logic
        if self.obstacle_detected or self.avoid_state is not None:
            self.handleObstacleAvoidance()
            return

        # Normal waypoint driving
        self.driveTowardWaypoint()

    def handleObstacleAvoidance(self) -> None:
        """
        Simple finite-state obstacle avoidance:
          - TURN LEFT while obstacle is detected.
          - Once clear, drive forward for 3 seconds,
            then resume normal navigation.
        """
        now = self.get_clock().now()

        # If obstacle is present
        if self.obstacle_detected:
            if self.avoid_state is None:
                self.avoid_state = "TURNING_LEFT"
                self.avoid_start_time = now
                self.get_logger().info(
                    colorStr("Obstacle detected - start turning left", ColorCodes.RED_ERR)
                )

            if self.avoid_state == "TURNING_LEFT":
                self.turnLeft()
                return

            if self.avoid_state == "DRIVING_FORWARD":
                # If new obstacle arrives while driving forward
                self.avoid_state = "TURNING_LEFT"
                self.avoid_start_time = now
                self.turnLeft()
                return
        else:
            # Obstacle not detected
            if self.avoid_state == "TURNING_LEFT":
                # Switch to driving forward once obstacle is gone
                self.avoid_state = "DRIVING_FORWARD"
                self.avoid_start_time = now
                self.get_logger().info(
                    colorStr(
                        "Finished turning, drive forward to bypass obstacle", ColorCodes.RED_ERR
                    )
                )

            if self.avoid_state == "DRIVING_FORWARD":
                # Drive forward for 3 seconds
                elapsed = (now - self.avoid_start_time).nanoseconds / 1e9
                if elapsed < 3.0:
                    self.driveStraight(0.25)
                    return
                else:
                    # Done avoiding
                    self.avoid_state = None
                    self.get_logger().info(
                        colorStr(
                            "Done avoiding obstacle. Resuming normal navigation.",
                            ColorCodes.GREEN_OK,
                        )
                    )
                    return

        # If no condition matched, just stop as a fallback
        self.stopRover()

    def turnLeft(self, speed: float = 0.2) -> None:
        """
        Turn left in place at a given speed.
        """
        self.publishDriveCommands(-speed, speed)

    def driveStraight(self, speed: float) -> None:
        """
        Drive forward at a given speed on both sides.
        """
        self.publishDriveCommands(speed, speed)

    def driveTowardWaypoint(self) -> None:
        """
        Normal navigation logic: correct heading error, move forward to waypoint.
        """
        dx = self.nav_feedback.x
        dy = self.nav_feedback.y
        heading_error = self.nav_feedback.theta

        angular_tolerance = 0.2  # ~11 degrees
        base_linear_speed = 0.3
        base_turn_speed = 0.2

        distance = math.sqrt(dx * dx + dy * dy)

        # Turn in place if heading error too big
        if abs(heading_error) > angular_tolerance:
            turn_speed = base_turn_speed * self.sign(heading_error)
            self.publishDriveCommands(-turn_speed, turn_speed)
            return

        # Otherwise, go straight
        forward_speed = base_linear_speed
        # Optionally scale by distance if desired
        # forward_speed = min(base_linear_speed, distance * 0.1)

        self.publishDriveCommands(forward_speed, forward_speed)

    # --------------------------------------------------------------------------
    #   Helper Functions
    # --------------------------------------------------------------------------
    def stopRover(self) -> None:
        """Publish zero velocity."""
        self.publishDriveCommands(0.0, 0.0)

    def publishDriveCommands(self, left_speed: float, right_speed: float) -> None:
        """
        Publishes Float32 speeds to the drivebase.
        Positive => forward, negative => reverse.
        """
        self.left_drive_pub.publish(Float32(data=left_speed))
        self.right_drive_pub.publish(Float32(data=right_speed))

    @staticmethod
    def sign(value: float) -> float:
        """
        Returns +1.0 if value >= 0.0, else -1.0.
        """
        return 1.0 if value >= 0.0 else -1.0


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
            decision_making_node.get_logger().info(
                colorStr("Shutting down decision_making_node", ColorCodes.BLUE_OK)
            )
    finally:
        if decision_making_node is not None:
            decision_making_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
