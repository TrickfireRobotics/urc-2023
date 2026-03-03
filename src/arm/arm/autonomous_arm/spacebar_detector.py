from typing import Optional

import cv2
import numpy as np

from src.lib.vision.aruco_detection import ArucoFrame


class SpacebarDetector:
    """
    Detects the spacebar key on the keyboard using contour analysis.
    Returns the spacebar center in keyboard-frame coordinates.
    """

    WARP_WIDTH = 600
    WARP_HEIGHT = 400

    def __init__(self) -> None:
        pass

    def _warp_keyboard_region(
        self, frame: np.ndarray, aruco_frame: ArucoFrame
    ) -> np.ndarray:
        """Perspective-warp the ArUco bounding box into a rectangular image."""
        src_pts = aruco_frame.bounding_box.astype(np.float32)
        # Order: top-left, top-right, bottom-right, bottom-left
        dst_pts = np.array(
            [
                [0, 0],
                [self.WARP_WIDTH, 0],
                [self.WARP_WIDTH, self.WARP_HEIGHT],
                [0, self.WARP_HEIGHT],
            ],
            dtype=np.float32,
        )
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return cv2.warpPerspective(frame, M, (self.WARP_WIDTH, self.WARP_HEIGHT))

    def detect_spacebar(
        self, frame: np.ndarray, aruco_frame: ArucoFrame
    ) -> Optional[tuple[float, float]]:
        """
        Detect the spacebar in the given frame and return its center
        in keyboard-frame coordinates. Returns None if not found.
        """
        return None
