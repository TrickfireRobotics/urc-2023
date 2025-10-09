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
    return NavigationNode()
