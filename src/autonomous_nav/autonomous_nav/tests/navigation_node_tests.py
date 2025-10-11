import math
import sys
from pathlib import Path
from typing import Tuple

import numpy as np
import pytest

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from nav_msgs.msg import OccupancyGrid
from navigation_node import NavigationNode


@pytest.fixture
def mock_costmap() -> OccupancyGrid:
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

    return mock_occupancy_grid


@pytest.fixture
def navigation_node(mock_costmap: OccupancyGrid) -> NavigationNode:
    test_navigation_node = NavigationNode()
    reached_threshold = 1.0  # meters
    earth_radius = 6371000.0  # Approx Earth radius in meters

    # ---- Anchor State (from /anchor_position) ----
    anchor_received = False
    ref_lat = 0.0
    ref_lon = 0.0
    ref_alt = 0.0
    start_lat = 0.0
    start_lon = 0.0
    start_alt = 0.0

    # ---- Internal State ----
    active_waypoint = (4, 3)
    current_position = (0.0, 0.0)  # x, y
    current_yaw = 0.0
    current_global_yaw = 0.0
    current_lat = 0.0
    current_lon = 0.0
    current_alt = 0.0
    end_goal_waypoint = (4, 3)
    global_costmap = mock_costmap

    return test_navigation_node


class TestNavigationNode:
    def test_plan_path(self, navigation_node: NavigationNode) -> None:
        test_path = navigation_node.planPath(navigation_node.global_costmap)
        assert hasattr(navigation_node, "path")
        assert isinstance(navigation_node.path.poses, list)

    def test_collect_radius(self, navigation_node: NavigationNode) -> None:
        # collect the radius from the center of the costmap
        costmap_center = int(
            navigation_node.global_costmap.info.width
            * navigation_node.global_costmap.info.height
            / 2
        )
        test_radius = navigation_node.collect_radius(
            navigation_node.global_costmap, costmap_center, 2
        )
        assert len(test_radius) > 1
        # should get a 2x2 box /0.5 (total of 80 indicies)
        # check that the points have the expected indicies
        # do this by checking if the center node is in the list

    def test_position_to_index(self, navigation_node: NavigationNode) -> None:
        index = navigation_node.position_to_index(navigation_node.global_costmap, (0, 0))
        assert index > 0

    def test_index_to_position(self, navigation_node: NavigationNode) -> None:
        position = navigation_node.index_to_position(navigation_node.global_costmap, 40)
        assert position[0] > 0
        assert position[1] > 0
