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
    MIN_ASPECT_RATIO = 2.0  # spacebar is at least 2x wider than tall
    MIN_AREA_FRACTION = 0.01  # spacebar must be at least 1% of the cropped image area

    def __init__(self) -> None:
        pass

    def _warp_keyboard_region(self, frame: np.ndarray, aruco_frame: ArucoFrame) -> np.ndarray:
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

    def _find_spacebar_contour(self, cropped: np.ndarray) -> Optional[np.ndarray]:
        """Find the spacebar contour: the widest, high-aspect-ratio rectangle."""
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11,
            -2,
        )
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        img_area = cropped.shape[0] * cropped.shape[1]
        best_contour = None
        best_width = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_AREA_FRACTION * img_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if h == 0:
                continue
            aspect = w / h
            if aspect >= self.MIN_ASPECT_RATIO and w > best_width:
                best_width = w
                best_contour = cnt

        return best_contour

    def detect_spacebar(
        self, frame: np.ndarray, aruco_frame: ArucoFrame
    ) -> Optional[tuple[float, float]]:
        """
        Detect the spacebar in the given frame and return its center
        in keyboard-frame coordinates. Returns None if not found.
        """
        warped = self._warp_keyboard_region(frame, aruco_frame)
        contour = self._find_spacebar_contour(warped)
        if contour is None:
            return None

        M = cv2.moments(contour)
        if M["m00"] == 0:
            return None

        px = M["m10"] / M["m00"]
        py = M["m01"] / M["m00"]

        x = (px / self.WARP_WIDTH) * aruco_frame.width
        y = (py / self.WARP_HEIGHT) * aruco_frame.height
        return (x, y)
