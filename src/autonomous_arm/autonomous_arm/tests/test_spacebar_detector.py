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


def _make_keyboard_image_with_spacebar(
    image_size: tuple[int, int] = (400, 500),
    spacebar_rect: tuple[int, int, int, int] = (150, 300, 200, 30),
) -> np.ndarray:
    """
    Draw a synthetic keyboard image with a white spacebar rectangle
    on a dark background. spacebar_rect is (x, y, width, height).
    """
    img = np.full((image_size[0], image_size[1], 3), 40, dtype=np.uint8)
    x, y, w, h = spacebar_rect
    cv2.rectangle(img, (x, y), (x + w, y + h), (200, 200, 200), -1)
    # Add some small keys above to ensure filtering works
    for i in range(5):
        kx = 100 + i * 40
        cv2.rectangle(img, (kx, 200), (kx + 25, 225), (200, 200, 200), -1)
    return img


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


class TestFindSpacebarContour:
    def test_finds_widest_contour(self) -> None:
        img = _make_keyboard_image_with_spacebar()
        detector = SpacebarDetector()
        contour = detector._find_spacebar_contour(img)
        assert contour is not None

    def test_returns_none_on_blank_image(self) -> None:
        blank = np.full((400, 500, 3), 40, dtype=np.uint8)
        detector = SpacebarDetector()
        contour = detector._find_spacebar_contour(blank)
        assert contour is None

    def test_spacebar_contour_is_widest(self) -> None:
        img = _make_keyboard_image_with_spacebar()
        detector = SpacebarDetector()
        contour = detector._find_spacebar_contour(img)
        x, y, w, h = cv2.boundingRect(contour)
        # Spacebar is 200px wide, small keys are 25px wide
        assert w > 100


def _make_frame_with_spacebar_in_aruco_region() -> tuple[np.ndarray, ArucoFrame]:
    """
    Create a 500x600 image where the ArUco region (50,50)-(350,250)
    contains a keyboard with a spacebar drawn inside it.
    """
    frame = np.full((400, 500, 3), 40, dtype=np.uint8)
    # Draw spacebar inside the aruco bounding region
    # Spacebar centered at roughly x=200, y=220 in the full frame
    cv2.rectangle(frame, (130, 210), (270, 240), (200, 200, 200), -1)
    # Small keys
    for i in range(5):
        kx = 100 + i * 40
        cv2.rectangle(frame, (kx, 150), (kx + 25, 170), (200, 200, 200), -1)

    aruco_frame = _make_aruco_frame(width=300.0, height=200.0)
    return frame, aruco_frame


class TestDetectSpacebar:
    def test_returns_tuple_on_valid_input(self) -> None:
        frame, aruco_frame = _make_frame_with_spacebar_in_aruco_region()
        detector = SpacebarDetector()
        result = detector.detect_spacebar(frame, aruco_frame)
        assert result is not None
        assert len(result) == 2

    def test_coordinates_within_frame_bounds(self) -> None:
        frame, aruco_frame = _make_frame_with_spacebar_in_aruco_region()
        detector = SpacebarDetector()
        result = detector.detect_spacebar(frame, aruco_frame)
        assert result is not None
        x, y = result
        assert 0 <= x <= aruco_frame.width
        assert 0 <= y <= aruco_frame.height

    def test_spacebar_in_lower_half(self) -> None:
        """The spacebar is at the bottom of the keyboard, so y > height/2."""
        frame, aruco_frame = _make_frame_with_spacebar_in_aruco_region()
        detector = SpacebarDetector()
        result = detector.detect_spacebar(frame, aruco_frame)
        assert result is not None
        _, y = result
        assert y > aruco_frame.height * 0.5

    def test_returns_none_on_blank_frame(self) -> None:
        frame = np.full((400, 500, 3), 40, dtype=np.uint8)
        aruco_frame = _make_aruco_frame()
        detector = SpacebarDetector()
        result = detector.detect_spacebar(frame, aruco_frame)
        assert result is None
