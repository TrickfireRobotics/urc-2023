import math
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dwa_planner import DWAPlanner, Trajectory
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid


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


def dwa_planner(mock_costmap: PyCostmap2D) -> DWAPlanner:
    """Create a DWAPlanner instance for testing."""
    return DWAPlanner(
        costmap=mock_costmap,
        robot_radius=0.3,
        current_velocity=(1.5, 1.0),  # (left, right) wheel velocities
        current_position=(0.0, 0.0),
        time_delta=0.1,
        goal=(5.0, 5.0),
        theta=0.0,
    )


def main() -> None:
    planner: DWAPlanner = dwa_planner(mock_costmap())

    window: List[Tuple[float, float]] = planner.calculate_dynamic_window(planner.current_wheel_vel)

    for velocity in window:
        print(f"Velocity Sample: Left: {velocity[0]:.2f} m/s, Right: {velocity[1]:.2f} m/s")
        print(velocity)

    vel_command: Tuple[float, float] = planner.plan()
    print(
        f"Velocity Command: Linear: {vel_command[0]:.2f} m/s, Angular: {vel_command[1]:.2f} rad/s"
    )


if __name__ == "__main__":
    main()
