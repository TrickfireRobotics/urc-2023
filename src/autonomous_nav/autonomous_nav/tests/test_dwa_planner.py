import math
import sys
from pathlib import Path

import pytest

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dwa_planner import DWAPlanner, Trajectory
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid


# Fixtures for pytest
@pytest.fixture
def mock_costmap() -> PyCostmap2D:
    """Create a mock costmap for testing."""
    mock_occupancy_grid = OccupancyGrid()
    # Set up a basic grid with some dimensions
    mock_occupancy_grid.info.resolution = 0.05  # 5cm per cell
    mock_occupancy_grid.info.width = 100
    mock_occupancy_grid.info.height = 100
    mock_occupancy_grid.info.origin.position.x = -2.5
    mock_occupancy_grid.info.origin.position.y = -2.5

    # Initialize with free space (0 = free)
    mock_occupancy_grid.data = [0] * (100 * 100)

    return PyCostmap2D(mock_occupancy_grid)


@pytest.fixture
def dwa_planner(mock_costmap: PyCostmap2D) -> DWAPlanner:
    """Create a DWAPlanner instance for testing."""
    return DWAPlanner(
        costmap=mock_costmap,
        robot_radius=0.3,
        current_velocity=(1.0, 1.0),  # (left, right) wheel velocities
        current_position=(0.0, 0.0),
        time_delta=0.1,
        goal=(5.0, 5.0),
        theta=0.0,
    )


# Test Trajectory class
class TestTrajectory:
    """Test the Trajectory class."""

    def test_trajectory_initialization(self) -> None:
        """Test that trajectory initializes correctly."""
        traj = Trajectory(linear_vel=1.0, angular_vel=0.5)
        assert traj.linear_vel == 1.0
        assert traj.angular_vel == 0.5
        assert traj.points == []
        assert traj.cost == float("inf")

    def test_add_point(self) -> None:
        """Test adding points to trajectory."""
        traj = Trajectory()
        traj.add_point(1.0, 2.0, 0.5)
        assert len(traj.points) == 1
        assert traj.points[0] == (1.0, 2.0, 0.5)

        traj.add_point(2.0, 3.0, 1.0)
        assert len(traj.points) == 2

    def test_normalize_angle(self) -> None:
        """Test angle normalization."""
        # Test positive angle wrapping
        assert abs(Trajectory.normalize_angle(3.5 * math.pi) - (-0.5 * math.pi)) < 0.001

        # Test negative angle wrapping
        assert abs(Trajectory.normalize_angle(-3.5 * math.pi) - (0.5 * math.pi)) < 0.001

        # Test angle already in range
        assert abs(Trajectory.normalize_angle(math.pi / 2) - (math.pi / 2)) < 0.001


# Test DWAPlanner class
class TestDWAPlanner:
    """Test the DWAPlanner class."""

    def test_planner_initialization(self, dwa_planner: DWAPlanner) -> None:
        """Test that planner initializes with correct parameters."""
        assert dwa_planner.robot_radius == 0.3
        assert dwa_planner.current_pos == (0.0, 0.0)
        assert dwa_planner.goal == (5.0, 5.0)
        assert dwa_planner.current_theta == 0.0
        assert dwa_planner.time_delta == 0.1

    def test_wheel_to_robot_velocities(self, dwa_planner: DWAPlanner) -> None:
        """Test conversion from wheel velocities to robot velocities."""
        # Test equal wheel velocities (straight motion)
        linear, angular = dwa_planner.wheel_to_robot_velocities((10.0, 10.0))
        assert abs(angular) < 0.001  # Should be nearly zero
        assert linear > 0  # Should be moving forward

        # Test differential velocities (turning motion)
        linear, angular = dwa_planner.wheel_to_robot_velocities((5.0, 10.0))
        assert angular > 0  # Should be turning
        assert linear > 0  # Should be moving forward

        # Test opposite wheel velocities (rotation in place)
        linear, angular = dwa_planner.wheel_to_robot_velocities((-5.0, 5.0))
        assert abs(linear) < 0.001  # Should be nearly zero linear velocity
        assert abs(angular) > 0  # Should be rotating

    def test_robot_to_wheel_velocities(self, dwa_planner: DWAPlanner) -> None:
        """Test conversion from robot velocities to wheel velocities."""
        # Test straight motion
        left, right = dwa_planner.robot_to_wheel_velocities(1.0, 0.0)
        assert abs(left - right) < 0.001  # Wheels should be equal

        # Test turning motion
        left, right = dwa_planner.robot_to_wheel_velocities(1.0, 0.5)
        assert right > left  # Right wheel should be faster for left turn

        # Test rotation in place
        left, right = dwa_planner.robot_to_wheel_velocities(0.0, 1.0)
        assert abs(left + right) < 0.001  # Wheels should be opposite

    def test_normalize_angle(self, dwa_planner: DWAPlanner) -> None:
        """Test angle normalization static method."""
        # Test wrapping from above pi
        assert abs(dwa_planner.normalize_angle(4 * math.pi) - 0.0) < 0.001

        # Test wrapping from below -pi
        assert abs(dwa_planner.normalize_angle(-4 * math.pi) - 0.0) < 0.001

        # Test angle in valid range
        assert abs(dwa_planner.normalize_angle(math.pi / 4) - (math.pi / 4)) < 0.001

    def test_simulate_trajectory_straight(self, dwa_planner: DWAPlanner) -> None:
        """Test trajectory simulation for straight motion."""
        trajectory = dwa_planner.simulate_trajectory(1.0, 0.0)

        # Check trajectory has points
        assert len(trajectory.points) > 0

        # Check first point is at starting position
        assert trajectory.points[0] == (0.0, 0.0, 0.0)

        # Check trajectory moves forward (x increases for theta=0)
        final_x, final_y, final_theta = trajectory.points[-1]
        assert final_x > 0  # Should move forward in x direction
        assert abs(final_y) < 0.1  # Should stay roughly on x-axis
        assert abs(final_theta) < 0.1  # Heading should remain ~0

    def test_simulate_trajectory_turning(self, dwa_planner: DWAPlanner) -> None:
        """Test trajectory simulation for turning motion."""
        trajectory = dwa_planner.simulate_trajectory(1.0, 0.5)

        # Check trajectory has points
        assert len(trajectory.points) > 0

        # Check trajectory curves (theta changes)
        final_x, final_y, final_theta = trajectory.points[-1]
        assert abs(final_theta) > 0.1  # Heading should have changed

        # Verify it's a curved path
        assert len(trajectory.points) > 2

    def test_calculate_dynamic_window(self, dwa_planner: DWAPlanner) -> None:
        """Test dynamic window calculation."""
        current_velocities = (1.0, 0.5)  # (linear, angular)
        min_v, max_v, min_w, max_w = dwa_planner.calculate_dynamic_window(current_velocities)

        # Check that window is valid
        assert min_v <= max_v
        assert min_w <= max_w

        # Check that current velocity is within window
        assert min_v <= current_velocities[0] <= max_v
        assert min_w <= current_velocities[1] <= max_w

        # Check that limits are respected
        assert max_v <= dwa_planner.max_linear_vel
        assert min_v >= dwa_planner.min_linear_vel
        assert max_w <= dwa_planner.max_angular_vel
        assert min_w >= -dwa_planner.max_angular_vel

    def test_generate_velocity_samples(self, dwa_planner: DWAPlanner) -> None:
        """Test velocity sample generation."""
        dynamic_window = (0.0, 1.0, -0.5, 0.5)
        samples = dwa_planner.generate_velocity_samples(dynamic_window)

        # Check that samples were generated
        assert len(samples) > 0

        # Check that all samples are within the dynamic window
        for linear, angular in samples:
            assert dynamic_window[0] <= linear <= dynamic_window[1]
            assert dynamic_window[2] <= angular <= dynamic_window[3]

        # Check expected number of samples
        expected_count = dwa_planner.linear_vel_samples * dwa_planner.angular_vel_samples
        assert len(samples) == expected_count

    def test_update_state(self, dwa_planner: DWAPlanner, mock_costmap: PyCostmap2D) -> None:
        """Test state update method."""
        new_position = (1.0, 1.0)
        new_theta = math.pi / 4
        new_velocity = (2.0, 2.0)
        new_goal = (10.0, 10.0)
        new_global_pose = (5.0, 5.0, math.pi / 4)

        dwa_planner.update_state(
            costmap=mock_costmap,
            current_position=new_position,
            current_theta=new_theta,
            current_velocity=new_velocity,
            goal=new_goal,
            global_pose=new_global_pose,
        )

        # Verify state was updated
        assert dwa_planner.current_pos == new_position
        assert dwa_planner.current_theta == new_theta
        assert dwa_planner.current_wheel_vel == new_velocity
        assert dwa_planner.goal == new_goal
        assert dwa_planner.global_pose == new_global_pose

    def test_plan_returns_wheel_velocities(self, dwa_planner: DWAPlanner) -> None:
        """Test that plan() returns normalized wheel velocities."""
        left, right = dwa_planner.plan()

        # Check that velocities are returned
        assert isinstance(left, (int, float))
        assert isinstance(right, (int, float))

        # Check that velocities are reasonable (not NaN or infinite)
        assert not math.isnan(left)
        assert not math.isnan(right)
        assert not math.isinf(left)
        assert not math.isinf(right)

        # Check that velocities are in normalized range
        # Outputs should be normalized for drivebase interface
        # Range: [min_normalized, 1.0] where min can be negative for reverse
        min_expected = (
            dwa_planner.min_linear_vel
            / dwa_planner.wheel_radius
            / dwa_planner.drivebase_speed_multiplier
        )
        assert (
            left >= min_expected and left <= 1.0
        ), f"Left velocity {left} out of range [{min_expected}, 1.0]"
        assert (
            right >= min_expected and right <= 1.0
        ), f"Right velocity {right} out of range [{min_expected}, 1.0]"

    def test_different_goals_produce_different_outputs(
        self, dwa_planner: DWAPlanner, mock_costmap: PyCostmap2D
    ) -> None:
        """Test that different goal positions result in different velocity commands."""
        # Store outputs for different goals
        outputs = []

        test_goals = [
            (2.0, 0.0),  # Straight ahead
            (2.0, 2.0),  # Forward and right
            (0.0, 2.0),  # Pure right
            (2.0, -2.0),  # Forward and left
        ]

        for goal in test_goals:
            dwa_planner.update_state(
                costmap=mock_costmap,
                current_position=(0.0, 0.0),
                current_theta=0.0,
                current_velocity=(1.0, 1.0),
                goal=goal,
                global_pose=(0.0, 0.0, 0.0),
            )
            output = dwa_planner.plan()
            outputs.append((goal, output))

        # Verify that not all outputs are the same
        unique_outputs = set(outputs[i][1] for i in range(len(outputs)))
        assert len(unique_outputs) > 1, f"All goals produced same output: {outputs[0][1]}"

        # Verify that forward goal produces forward motion
        forward_output = outputs[0][1]  # (2.0, 0.0) goal
        assert (
            forward_output[0] > 0 or forward_output[1] > 0
        ), "Forward goal should produce forward motion"

        # Verify that outputs are not all zeros (unless robot is stuck)
        non_zero_outputs = [out for _, out in outputs if out != (0.0, 0.0)]
        assert (
            len(non_zero_outputs) > 0
        ), "All outputs are zero - robot may be stuck or planner not working"


# Main function for running tests directly
def main() -> None:
    """Run tests using pytest."""
    pytest.main([__file__, "-v"])


if __name__ == "__main__":
    main()
