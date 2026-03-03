import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

# Add src/ to path for lib.vision imports, and src/arm/ for arm.autonomous_arm imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from arm.autonomous_arm.spacebar_detector import SpacebarDetector
from lib.vision.aruco_detection import ArucoFrame


def _make_aruco_frame(
    width: float = 300.0, height: float = 200.0
) -> ArucoFrame:
    """Create a fake ArucoFrame with a simple axis-aligned bounding box."""
    box = np.array(
        [[50, 50], [350, 50], [350, 250], [50, 250]], dtype=np.float32
    )
    return ArucoFrame(
        frame_id="aruco_frame_0",
        marker_ids=[0, 1, 2, 3],
        bounding_box=box,
        center=(200.0, 150.0),
        width=width,
        height=height,
        angle=0.0,
    )


class TestWarpKeyboardRegion:
    def test_returns_image_with_correct_shape(self) -> None:
        frame = np.full((400, 500, 3), 128, dtype=np.uint8)
        aruco_frame = _make_aruco_frame(width=300.0, height=200.0)
        detector = SpacebarDetector()
        warped = detector._warp_keyboard_region(frame, aruco_frame)
        assert warped.shape[0] > 0
        assert warped.shape[1] > 0
        assert len(warped.shape) == 3

    def test_warped_region_is_rectangular(self) -> None:
        frame = np.full((400, 500, 3), 128, dtype=np.uint8)
        aruco_frame = _make_aruco_frame(width=300.0, height=200.0)
        detector = SpacebarDetector()
        warped = detector._warp_keyboard_region(frame, aruco_frame)
        h, w = warped.shape[:2]
        assert w > 0
        assert h > 0
