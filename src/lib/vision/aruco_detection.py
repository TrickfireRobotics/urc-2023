from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class ArucoMarkerResult:
    marker_id: int
    frame_id: str  # e.g. "aruco_23"
    corners: np.ndarray  # 4 corner points in pixel coords
    center: tuple[float, float]  # pixel center (cx, cy)
    rvec: Optional[np.ndarray] = None  # rotation vector from pose estimation
    tvec: Optional[np.ndarray] = None  # translation vector from pose estimation


@dataclass
class ArucoFrame:
    frame_id: str  # e.g. "aruco_frame_0"
    marker_ids: list[int]  # IDs of the markers that define this frame
    bounding_box: np.ndarray  # 4 corners of the bounding rect in pixel coords (rotated rect)
    center: tuple[float, float]  # pixel center of the bounding box
    width: float  # width of the bounding box in pixels
    height: float  # height of the bounding box in pixels
    angle: float  # rotation angle of the bounding box in degrees


class ArucoDetection:
    def __init__(
        self,
        dict_type: int = cv2.aruco.DICT_6X6_250,
        marker_length: float = 0.0529,
        cam_id: int = 0,
    ) -> None:
        self.cam_id = cam_id
        self.marker_length = marker_length
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None

    def load_calibration(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> None:
        """Load camera intrinsics for pose estimation."""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def load_calibration_from_file(self, camera_matrix_path: str, dist_coeffs_path: str) -> None:
        """Load camera intrinsics from text files saved by the calibration pipeline."""
        self.camera_matrix = np.loadtxt(camera_matrix_path).astype(np.float32)
        self.dist_coeffs = np.loadtxt(dist_coeffs_path).astype(np.float32)

    def detect_markers(self, frame: np.ndarray) -> list[ArucoMarkerResult]:
        """
        Detect ArUco markers in a frame and estimate their poses if calibration is loaded.
        Returns a list of ArucoMarkerResult for each detected marker.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return []

        results: list[ArucoMarkerResult] = []
        for i, marker_id in enumerate(ids.flatten()):
            pts = corners[i][0]
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            result = ArucoMarkerResult(
                marker_id=int(marker_id),
                frame_id=f"aruco_{int(marker_id)}",
                corners=pts,
                center=(cx, cy),
            )

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                rvec, tvec = self._estimate_pose(pts)
                result.rvec = rvec
                result.tvec = tvec

            results.append(result)

        return results

    def _estimate_pose(self, corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Estimate pose of a single marker using solvePnP."""
        assert self.camera_matrix is not None and self.dist_coeffs is not None
        marker_points = np.array(
            [
                [-self.marker_length / 2, self.marker_length / 2, 0],
                [self.marker_length / 2, self.marker_length / 2, 0],
                [self.marker_length / 2, -self.marker_length / 2, 0],
                [-self.marker_length / 2, -self.marker_length / 2, 0],
            ],
            dtype=np.float32,
        )
        _, rvec, tvec = cv2.solvePnP(marker_points, corners, self.camera_matrix, self.dist_coeffs)
        return rvec, tvec

    def compute_frame(self, markers: list[ArucoMarkerResult]) -> Optional[ArucoFrame]:
        """
        Compute a bounding box frame from all detected markers.
        Uses the outermost corners of all markers to create a minimum area rotated rectangle.
        Returns None if fewer than 2 markers are provided.
        """
        if len(markers) < 2:
            return None

        all_corners = np.vstack([m.corners for m in markers])
        rect = cv2.minAreaRect(all_corners.astype(np.float32))
        box = cv2.boxPoints(rect)
        (cx, cy), (w, h), angle = rect

        return ArucoFrame(
            frame_id="aruco_frame_0",
            marker_ids=[m.marker_id for m in markers],
            bounding_box=box,
            center=(cx, cy),
            width=w,
            height=h,
            angle=angle,
        )

    @staticmethod
    def get_marker_transform(rvec: np.ndarray, tvec: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Convert OpenCV rvec/tvec to a rotation matrix and translation.
        Returns (rotation_matrix_3x3, translation_3x1).
        """
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec

    def draw_detections(self, frame: np.ndarray, markers: list[ArucoMarkerResult]) -> np.ndarray:
        """Draw detected marker outlines, IDs, and pose axes onto a frame."""
        output = frame.copy()
        if not markers:
            return output

        corners = [m.corners.reshape(1, 4, 2) for m in markers]
        ids = np.array([[m.marker_id] for m in markers])
        cv2.aruco.drawDetectedMarkers(output, corners, ids)

        for m in markers:
            cx, cy = int(m.center[0]), int(m.center[1])
            cv2.putText(
                output,
                f"ID:{m.marker_id}",
                (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            if m.rvec is not None and m.tvec is not None and self.camera_matrix is not None:
                cv2.drawFrameAxes(
                    output,
                    self.camera_matrix,
                    self.dist_coeffs,
                    m.rvec,
                    m.tvec,
                    self.marker_length,
                )

        return output

    def draw_frame(self, frame: np.ndarray, aruco_frame: ArucoFrame) -> np.ndarray:
        """Draw the bounding box frame onto an image."""
        output = frame.copy()
        box = aruco_frame.bounding_box.astype(np.intp)
        cv2.drawContours(output, [box], 0, (255, 0, 0), 2)
        cx, cy = int(aruco_frame.center[0]), int(aruco_frame.center[1])
        cv2.putText(
            output,
            aruco_frame.frame_id,
            (cx - 40, cy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
        )
        return output
