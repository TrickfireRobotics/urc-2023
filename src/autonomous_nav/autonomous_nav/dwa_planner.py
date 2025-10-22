import math
from typing import TYPE_CHECKING, List, Optional, Tuple

import numpy as np
from nav2_simple_commander.costmap_2d import PyCostmap2D

# if TYPE_CHECKING:
#     from nav2_simple_commander.costmap_2d import PyCostmap2D


class Trajectory:
    def __init__(
        self,
        linear_vel: float = 0.0,
        angular_vel: float = 0.0,
        points: Optional[List[Tuple[float, float, float]]] = None,
    ):
        """
        Initialize a trajectory.
        Points are stored as (x, y, theta) tuples.
        """
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.points = points if points is not None else []
        self.cost = float("inf")

        # Cost weights for different objectives - priority can be changed.
        self.obstacle_cost_weight = 10.0
        self.goal_cost_weight = 5.0
        self.heading_cost_weight = 2.0
        self.velocity_cost_weight = 1.0

    def add_point(self, x: float, y: float, theta: float) -> None:
        """Add a point to the trajectory."""
        self.points.append((x, y, theta))

    # EVALUATE BASED ON THE DISTANCE TO THE NEXT PATH POSE. CLOSER TO THE NEXT POINT MEANS LOWER
    # COST GIVEN THE ROVER IS AT THE CURRENT WAYPOINT
    def has_collision(
        self,
        costmap: PyCostmap2D,
        goal: Tuple[float, float],
        max_linear_vel: float,
        robot_radius: float,
        costmap_frame_origin: Tuple[float, float],
    ) -> bool:
        """
        Evaluate trajectory and compute its total cost.
        Returns False if trajectory collides with obstacles.

        Prioritize trajectories that make it to the current goal. If that is none, then prioritize
        trajectories that get closer to the goal with a bias for getting closer to the next goal.

        Args:
            costmap_frame_origin: Robot's global position (odom frame) where costmap is centered
        """
        if not self.points:
            return False

        # Get costmap metadata
        resolution = costmap.getResolution()  # meters per cell
        origin_x = costmap.getOriginX()  # costmap's origin in its frame
        origin_y = costmap.getOriginY()
        size_x = costmap.getSizeInCellsX()
        size_y = costmap.getSizeInCellsY()

        obstacle_cost = 0.0
        min_clearance = float("inf")

        # Check each point in trajectory for collision
        for x, y, _ in self.points:
            map_x = (x - origin_x) / resolution
            map_y = (y - origin_y) / resolution

            # Check bounds
            if not (0 <= map_x < size_x and 0 <= map_y < size_y):
                return False

            # Check rover footprint (circle approximation)
            footprint_checks = []
            num_angles = 8  # Check 8 points around rover circumference
            for angle_idx in range(num_angles):
                angle = 2 * math.pi * angle_idx / num_angles
                check_x = x + robot_radius * math.cos(angle)
                check_y = y + robot_radius * math.sin(angle)

                # Transform to map coordinates
                check_map_x = int((check_x - origin_x) / resolution)
                check_map_y = int((check_y - origin_y) / resolution)

                # Bounds check
                if 0 <= check_map_x < size_x and 0 <= check_map_y < size_y:
                    cost = float(costmap.getCostXY(check_map_x, check_map_y))
                    footprint_checks.append(cost)

                    # Lethal obstacle check - value to be set based on capabilites of the rover
                    if cost >= 253:
                        return False

                    # Accumulate obstacle proximity cost
                    if cost > 0:
                        obstacle_cost += cost
                        min_clearance = min(min_clearance, (252 - cost) / 252.0)

        # Average obstacle cost across trajectory (normalized to [0, 1])
        avg_obstacle_cost = obstacle_cost / len(self.points) if self.points else 0.0
        avg_obstacle_cost = avg_obstacle_cost / 252.0  # Normalize to [0, 1] - value can be changed

        # Goal distance cost
        final_x, final_y, final_theta = self.points[-1]
        goal_distance = math.sqrt((final_x - goal[0]) ** 2 + (final_y - goal[1]) ** 2)
        # Normalize by maximum distance robot could travel
        max_travel_distance = max_linear_vel * 2.0
        goal_cost = goal_distance / max(max_travel_distance, 0.1)

        # Heading cost (normalized to [0, 1] since heading_diff is in [0, π])
        goal_angle = math.atan2(goal[1] - final_y, goal[0] - final_x)
        heading_diff = abs(self.normalize_angle(goal_angle - final_theta))
        heading_cost = heading_diff / math.pi  # Normalize to [0, 1]

        # Velocity cost (prefer higher velocities) - lower cost for higher speed
        # This is correct: high speed → low cost → preferred
        velocity_cost = (max_linear_vel - abs(self.linear_vel)) / max_linear_vel

        # Total weighted cost
        self.cost = (
            self.obstacle_cost_weight * avg_obstacle_cost
            + self.goal_cost_weight * goal_cost
            + self.heading_cost_weight * heading_cost
            + self.velocity_cost_weight * velocity_cost
        )

        return True

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


"""" Might need major changes. Implementation is going to use rolling window costmap 2D. """


class DWAPlanner:
    def __init__(
        self,
        costmap: PyCostmap2D,
        robot_radius: float,
        current_velocity: Tuple[float, float],
        current_position: Tuple[float, float],
        time_delta: float,
        goal: Tuple[float, float],
        theta: float,
    ):
        """
        Initialize the DWA planner with robot parameters.
        """
        # State Variables
        self.costmap = costmap
        self.robot_radius = robot_radius
        self.current_wheel_vel = current_velocity  # (left, right) wheel velocities
        self.current_pos = current_position
        self.time_delta = time_delta
        self.goal = goal
        self.current_theta = theta
        self.global_pose = (0.0, 0.0, 0.0)

        # Robot physical parameters - rough estimates from rover
        self.wheel_base = 0.5
        self.wheel_radius = 0.11

        # Velocity limits (in m/s and rad/s)
        self.max_linear_vel = 1.0
        self.min_linear_vel = -0.5
        self.max_angular_vel = 1.0

        # Drivebase interface expects normalized values [0, 1] that it multiplies by SPEED
        # SPEED = 6.28 * 2.5 = 15.7 rad/s (from drivebase.py)
        self.drivebase_speed_multiplier = 6.28 * 2.5

        # Calculate maximum wheel velocity for normalization
        self.max_wheel_linear = self.max_linear_vel + (self.max_angular_vel * self.wheel_base / 2.0)
        self.max_wheel_angular_velocity = self.max_wheel_linear / self.wheel_radius

        # Acceleration limits (in m/s² and rad/s²)
        # Need specific data based on rover
        self.max_linear_accel = 2.0
        self.max_angular_accel = 3.0

        # DWA parameters
        self.prediction_time = 2.0
        self.linear_vel_samples = 11
        self.angular_vel_samples = 21

        # Minimum trajectory points for valid evaluation
        self.min_trajectory_points = 10

    def update_state(
        self,
        costmap: PyCostmap2D,
        current_position: Tuple[float, float],
        current_theta: float,
        current_velocity: Tuple[float, float],
        goal: Tuple[float, float],
        global_pose: Tuple[float, float, float],
    ) -> None:
        """
        Update planner state without recreating the object.
        Call this each planning cycle before plan().

        Args:
            global_pose: Robot's pose in odom/map frame (x, y, theta)
            current_position: Robot's position in local/costmap frame
            goal: Goal position (must match frame of current_position)
        """
        self.costmap = costmap
        self.current_pos = current_position
        self.current_theta = current_theta
        self.current_wheel_vel = current_velocity
        self.goal = goal
        self.global_pose = global_pose

    def plan(self) -> Tuple[float, float]:
        """
        Main planning method - generates trajectories and returns best wheel velocities.

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Convert current wheel velocities to robot velocities
        current_velocities: Tuple[float, float] = self.wheel_to_robot_velocities(
            self.current_wheel_vel
        )

        # Generate dynamic window to create velocity samples
        dynamic_window = self.calculate_dynamic_window(current_velocities)

        # Get velocity samples
        velocity_samples = self.generate_velocity_samples(dynamic_window)

        # Generate and evaluate all trajectory candidates
        if self.costmap is None:
            return (0.0, 0.0)
        
        valid_trajectories = []
        for linear_vel, angular_vel in velocity_samples:
            trajectory = self.simulate_trajectory(linear_vel, angular_vel)

            # Evaluate trajectory (computes cost and checks collisions)
            if trajectory.has_collision(
                self.costmap,
                self.goal,
                self.max_linear_vel,
                self.robot_radius,
                costmap_frame_origin=(self.global_pose[0], self.global_pose[1]),
            ):
                valid_trajectories.append(trajectory)

        if not valid_trajectories:
            # Emergency stop if no valid trajectories
            return (0.0, 0.0)

        # Select the best trajectory (lowest cost)
        best_trajectory = min(valid_trajectories, key=lambda t: t.cost)

        # Convert the best trajectory's velocities back to wheel velocities (rad/s)
        left_wheel, right_wheel = self.robot_to_wheel_velocities(
            best_trajectory.linear_vel, best_trajectory.angular_vel
        )

        # Normalize to [0, 1] range for drivebase interface
        # Drivebase will multiply by SPEED to get back to rad/s
        left_normalized = left_wheel / self.drivebase_speed_multiplier
        right_normalized = right_wheel / self.drivebase_speed_multiplier

        # Clamp to valid range [min_vel_normalized, 1.0]
        # Allow negative for reverse motion
        min_normalized = self.min_linear_vel / self.wheel_radius / self.drivebase_speed_multiplier
        left_clamped = max(min_normalized, min(1.0, left_normalized))
        right_clamped = max(min_normalized, min(1.0, right_normalized))

        return (left_clamped, right_clamped)

    def wheel_to_robot_velocities(self, wheel_vels: Tuple[float, float]) -> Tuple[float, float]:
        """
        Convert wheel velocities to robot velocities.

        Args:
            wheel_vels: (left_wheel_vel, right_wheel_vel) in rad/s

        Returns:
            (linear_velocity, angular_velocity) in m/s and rad/s
        """
        left_vel, right_vel = wheel_vels

        # Convert wheel angular velocities to linear velocities
        left_linear = left_vel * self.wheel_radius
        right_linear = right_vel * self.wheel_radius

        # Calculate robot velocities using differential drive kinematics
        linear_vel = (right_linear + left_linear) / 2.0
        angular_vel = (right_linear - left_linear) / self.wheel_base

        return (linear_vel, angular_vel)

    def robot_to_wheel_velocities(
        self, linear_vel: float, angular_vel: float
    ) -> Tuple[float, float]:
        """
        Convert robot velocities to wheel velocities.

        Args:
            linear_vel: Forward velocity in m/s
            angular_vel: Angular velocity in rad/s

        Returns:
            (left_wheel_vel, right_wheel_vel) in rad/s
        """
        # Calculate wheel linear velocities
        left_linear = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_linear = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # Convert to wheel angular velocities
        left_wheel = left_linear / self.wheel_radius
        right_wheel = right_linear / self.wheel_radius

        return (left_wheel, right_wheel)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def simulate_trajectory(self, linear_vel: float, angular_vel: float) -> Trajectory:
        """
        Simulate a trajectory for given velocities using differential drive kinematics.

        Args:
            linear_vel: Forward velocity in m/s
            angular_vel: Angular velocity in rad/s

        Returns:
            Trajectory object containing predicted path
        """
        trajectory = Trajectory(linear_vel, angular_vel)

        # Start from current state
        x, y = self.current_pos
        theta = self.current_theta

        # Add starting point
        trajectory.add_point(x, y, theta)

        # Number of simulation steps
        num_steps = int(self.prediction_time / self.time_delta)

        # Simulate forward using differential drive kinematics
        for _ in range(num_steps):
            if abs(angular_vel) < 0.001:  # Nearly straight motion
                # Use simplified model for straight motion
                x_new = x + linear_vel * self.time_delta * math.cos(theta)
                y_new = y + linear_vel * self.time_delta * math.sin(theta)
                theta_new = theta
            else:  # Curved motion
                # Calculate radius of curvature
                radius = linear_vel / angular_vel

                # Calculate new position using arc motion
                x_new = x + radius * (
                    math.sin(theta + angular_vel * self.time_delta) - math.sin(theta)
                )
                y_new = y - radius * (
                    math.cos(theta + angular_vel * self.time_delta) - math.cos(theta)
                )
                theta_new = theta + angular_vel * self.time_delta

            # Normalize theta to [-pi, pi]
            theta_new = self.normalize_angle(theta_new)

            # Update state for next iteration
            x, y, theta = x_new, y_new, theta_new

            # Add point to trajectory
            trajectory.add_point(x, y, theta)

        return trajectory

    def calculate_dynamic_window(
        self, current_velocities: Tuple[float, float]
    ) -> Tuple[float, float, float, float]:
        """
        Calculate the dynamic window based on current velocities and acceleration limits.

        Returns:
            (min_linear_vel, max_linear_vel, min_angular_vel, max_angular_vel)
        """
        current_linear, current_angular = current_velocities

        # Calculate reachable velocities within one time step
        min_linear = max(
            current_linear - self.max_linear_accel * self.time_delta, self.min_linear_vel
        )
        max_linear = min(
            current_linear + self.max_linear_accel * self.time_delta, self.max_linear_vel
        )

        # Angular velocity window
        min_angular = max(
            current_angular - self.max_angular_accel * self.time_delta, -self.max_angular_vel
        )
        max_angular = min(
            current_angular + self.max_angular_accel * self.time_delta, self.max_angular_vel
        )

        return (min_linear, max_linear, min_angular, max_angular)

    def generate_velocity_samples(
        self, dynamic_window: Tuple[float, float, float, float]
    ) -> List[Tuple[float, float]]:
        """
        Generate velocity samples within the dynamic window.

        Returns:
            List of (linear_vel, angular_vel) tuples to evaluate
        """
        # Get dynamic window bounds
        min_v, max_v, min_w, max_w = dynamic_window

        velocity_samples = []

        # Sample velocities uniformly within the window
        if max_v > min_v:
            linear_samples = np.linspace(min_v, max_v, self.linear_vel_samples)
        else:
            linear_samples = np.array([min_v])

        if max_w > min_w:
            angular_samples = np.linspace(min_w, max_w, self.angular_vel_samples)
        else:
            angular_samples = np.array([min_w])

        # Create all combinations
        for v in linear_samples:
            for w in angular_samples:
                velocity_samples.append((v, w))

        return velocity_samples
