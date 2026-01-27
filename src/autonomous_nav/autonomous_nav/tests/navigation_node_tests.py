import math
import sys
from pathlib import Path as filePath
from typing import Generator, Tuple

import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Path


@pytest.fixture(scope="session", autouse=True)
def ros2_init_shutdown() -> Generator:
    rclpy.init()
    yield
    rclpy.shutdown()


# Add parent directory to path for imports
sys.path.insert(0, str(filePath(__file__).parent.parent))

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
    mock_occupancy_grid.info.resolution = 0.05

    # Initialize with free space (0 = free)
    mock_occupancy_grid.data = [0] * (100 * 100)

    return mock_occupancy_grid


@pytest.fixture
def navigation_node(mock_costmap: OccupancyGrid) -> NavigationNode:
    test_navigation_node = NavigationNode()
    test_navigation_node.reached_threshold = 1.0  # meters
    test_navigation_node.earth_radius = 6371000.0  # Approx Earth radius in meters

    # ---- Anchor State (from /anchor_position) ----
    test_navigation_node.anchor_received = False
    test_navigation_node.ref_lat = 0.0
    test_navigation_node.ref_lon = 0.0
    test_navigation_node.ref_alt = 0.0
    test_navigation_node.start_lat = 0.0
    test_navigation_node.start_lon = 0.0
    test_navigation_node.start_alt = 0.0

    # ---- Internal State ----
    test_navigation_node.active_waypoint = (4, 3)
    test_navigation_node.current_position = (0.0, 0.0)  # x, y
    test_navigation_node.current_yaw = 0.0
    test_navigation_node.current_global_yaw = 0.0
    test_navigation_node.current_lat = 0.0
    test_navigation_node.current_lon = 0.0
    test_navigation_node.current_alt = 0.0
    test_navigation_node.end_goal_waypoint = (4, 3)
    test_navigation_node.global_costmap = mock_costmap
    test_navigation_node.path = Path()
    test_navigation_node.path.poses = []
    return test_navigation_node


class TestNavigationNode:
    def test_collect_radius(self, navigation_node: NavigationNode) -> None:

        costmap_center = int(
            navigation_node.global_costmap.info.width
            * navigation_node.global_costmap.info.height
            / 2
        )
        test_radius = navigation_node.collect_radius(
            navigation_node.global_costmap, costmap_center, 2
        )
        assert len(test_radius) > 1

    def test_position_to_index(self, navigation_node: NavigationNode) -> None:
        index = navigation_node.position_to_index(navigation_node.global_costmap, (0, 0))
        assert index > 0

    def test_index_to_position(self, navigation_node: NavigationNode) -> None:
        position = navigation_node.index_to_position(navigation_node.global_costmap, 0)
        assert position[0] == navigation_node.global_costmap.info.origin.position.x
        assert position[1] == navigation_node.global_costmap.info.origin.position.y
        position2 = navigation_node.index_to_position(navigation_node.global_costmap, 20)
        # assert position2[0] == navigation_node.global_costmap.info.origin.position.x
        assert position2[1] == position[1]

    def test_find_cheapest(self, navigation_node: NavigationNode) -> None:
        # test_list = [(0, 5), (0, 4), (0, 3)]
        costmap_center = int(
            navigation_node.global_costmap.info.width
            * navigation_node.global_costmap.info.height
            / 2
        )
        test_radius = navigation_node.collect_radius(
            navigation_node.global_costmap, costmap_center, 2
        )
        cheapest_node = navigation_node.find_lowest_cost_node(
            test_radius, navigation_node.global_costmap
        )
        assert cheapest_node != (0, 0)

    def test_plan_path(self, navigation_node: NavigationNode) -> None:
        test_path = navigation_node.planPath(navigation_node.global_costmap)
        assert hasattr(navigation_node, "path")
        assert isinstance(navigation_node.path.poses, list)
