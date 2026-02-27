import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

# Add parent directories to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from vision.aruco_detection import ArucoDetection, ArucoFrame, ArucoMarkerResult

MARKER_LENGTH = 0.05
DICT_TYPE = cv2.aruco.DICT_6X6_250
# Reasonable fake calibration for a 640x480 image
CAMERA_MATRIX = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)


def _make_marker_image(marker_id: int, marker_px: int = 200, image_size: int = 640) -> np.ndarray:
    """Generate a BGR image with a single ArUco marker centred in frame."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
    marker = np.zeros((marker_px, marker_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_px, marker, 1)

    # Place marker on a white background
    canvas = np.full((image_size, image_size, 3), 255, dtype=np.uint8)
    y_off = (image_size - marker_px) // 2
    x_off = (image_size - marker_px) // 2
    canvas[y_off : y_off + marker_px, x_off : x_off + marker_px] = cv2.cvtColor(
        marker, cv2.COLOR_GRAY2BGR
    )
    return canvas


def _make_two_marker_image(
    id_a: int, id_b: int, marker_px: int = 150, image_size: int = 640
) -> np.ndarray:
    """Generate a BGR image with two ArUco markers side by side."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
    canvas = np.full((image_size, image_size, 3), 255, dtype=np.uint8)

    for i, mid in enumerate((id_a, id_b)):
        marker = np.zeros((marker_px, marker_px), dtype=np.uint8)
        cv2.aruco.generateImageMarker(aruco_dict, mid, marker_px, marker, 1)
        y_off = (image_size - marker_px) // 2
        x_off = 80 + i * (marker_px + 80)
        canvas[y_off : y_off + marker_px, x_off : x_off + marker_px] = cv2.cvtColor(
            marker, cv2.COLOR_GRAY2BGR
        )
    return canvas


# ── Fixtures ─────────────────────────────────────────────────────────────


@pytest.fixture
def detector() -> ArucoDetection:
    """ArucoDetection without calibration loaded."""
    return ArucoDetection(dict_type=DICT_TYPE, marker_length=MARKER_LENGTH)


@pytest.fixture
def calibrated_detector() -> ArucoDetection:
    """ArucoDetection with calibration loaded."""
    det = ArucoDetection(dict_type=DICT_TYPE, marker_length=MARKER_LENGTH)
    det.load_calibration(CAMERA_MATRIX, DIST_COEFFS)
    return det


@pytest.fixture
def single_marker_image() -> np.ndarray:
    return _make_marker_image(marker_id=23)


@pytest.fixture
def two_marker_image() -> np.ndarray:
    return _make_two_marker_image(id_a=1, id_b=5)


# ── Initialization ──────────────────────────────────────────────────────


class TestInit:
    def test_defaults(self) -> None:
        det = ArucoDetection()
        assert det.marker_length == 0.0529
        assert det.camera_matrix is None
        assert det.dist_coeffs is None

    def test_custom_parameters(self) -> None:
        det = ArucoDetection(dict_type=cv2.aruco.DICT_4X4_50, marker_length=0.1, cam_id=2)
        assert det.marker_length == 0.1
        assert det.cam_id == 2


# ── Calibration Loading ─────────────────────────────────────────────────


class TestLoadCalibration:
    def test_load_calibration(self, detector: ArucoDetection) -> None:
        detector.load_calibration(CAMERA_MATRIX, DIST_COEFFS)
        assert detector.camera_matrix is not None
        assert detector.dist_coeffs is not None
        np.testing.assert_array_equal(detector.camera_matrix, CAMERA_MATRIX)

    def test_load_calibration_from_file(self, detector: ArucoDetection, tmp_path: Path) -> None:
        mtx_path = tmp_path / "camera_matrix.txt"
        dist_path = tmp_path / "dist_coeffs.txt"
        np.savetxt(str(mtx_path), CAMERA_MATRIX)
        np.savetxt(str(dist_path), DIST_COEFFS)

        detector.load_calibration_from_file(str(mtx_path), str(dist_path))
        assert detector.camera_matrix is not None
        np.testing.assert_array_almost_equal(detector.camera_matrix, CAMERA_MATRIX)


# ── Marker Detection ────────────────────────────────────────────────────


class TestDetectMarkers:
    def test_detects_single_marker(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        assert len(markers) == 1
        assert markers[0].marker_id == 23

    def test_detects_two_markers(
        self, detector: ArucoDetection, two_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(two_marker_image)
        assert len(markers) == 2
        detected_ids = sorted(m.marker_id for m in markers)
        assert detected_ids == [1, 5]

    def test_returns_empty_on_blank_image(self, detector: ArucoDetection) -> None:
        blank = np.full((480, 640, 3), 200, dtype=np.uint8)
        markers = detector.detect_markers(blank)
        assert markers == []

    def test_marker_result_fields(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        m = markers[0]
        assert isinstance(m, ArucoMarkerResult)
        assert m.frame_id == "aruco_23"
        assert m.corners.shape == (4, 2)
        assert len(m.center) == 2

    def test_center_is_within_image(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        h, w = single_marker_image.shape[:2]
        cx, cy = markers[0].center
        assert 0 <= cx <= w
        assert 0 <= cy <= h

    def test_no_pose_without_calibration(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        assert markers[0].rvec is None
        assert markers[0].tvec is None


# ── Pose Estimation ─────────────────────────────────────────────────────


class TestPoseEstimation:
    def test_pose_estimated_with_calibration(
        self, calibrated_detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = calibrated_detector.detect_markers(single_marker_image)
        m = markers[0]
        assert m.rvec is not None
        assert m.tvec is not None

    def test_rvec_tvec_shapes(
        self, calibrated_detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = calibrated_detector.detect_markers(single_marker_image)
        m = markers[0]
        assert m.rvec.shape == (3, 1)
        assert m.tvec.shape == (3, 1)

    def test_tvec_z_positive(
        self, calibrated_detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        """Marker is in front of the camera so z should be positive."""
        markers = calibrated_detector.detect_markers(single_marker_image)
        assert markers[0].tvec[2] > 0


# ── get_marker_transform ────────────────────────────────────────────────


class TestGetMarkerTransform:
    def test_identity_rotation(self) -> None:
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.array([[1.0], [2.0], [3.0]], dtype=np.float64)
        R, t = ArucoDetection.get_marker_transform(rvec, tvec)
        np.testing.assert_array_almost_equal(R, np.eye(3))
        np.testing.assert_array_equal(t, tvec)

    def test_rotation_matrix_is_orthogonal(
        self, calibrated_detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = calibrated_detector.detect_markers(single_marker_image)
        R, _ = ArucoDetection.get_marker_transform(markers[0].rvec, markers[0].tvec)
        # R^T @ R should be identity
        np.testing.assert_array_almost_equal(R.T @ R, np.eye(3), decimal=5)


# ── compute_frame ────────────────────────────────────────────────────────


class TestComputeFrame:
    def test_returns_none_for_single_marker(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        assert len(markers) == 1
        assert detector.compute_frame(markers) is None

    def test_returns_none_for_empty_list(self, detector: ArucoDetection) -> None:
        assert detector.compute_frame([]) is None

    def test_returns_frame_for_two_markers(
        self, detector: ArucoDetection, two_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(two_marker_image)
        frame = detector.compute_frame(markers)
        assert frame is not None
        assert isinstance(frame, ArucoFrame)

    def test_frame_fields(
        self, detector: ArucoDetection, two_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(two_marker_image)
        frame = detector.compute_frame(markers)
        assert frame.frame_id == "aruco_frame_0"
        assert sorted(frame.marker_ids) == [1, 5]
        assert frame.bounding_box.shape == (4, 2)
        assert frame.width > 0
        assert frame.height > 0


# ── draw_detections ──────────────────────────────────────────────────────


class TestDrawDetections:
    def test_returns_copy_not_original(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        output = detector.draw_detections(single_marker_image, markers)
        assert output is not single_marker_image

    def test_output_same_shape(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(single_marker_image)
        output = detector.draw_detections(single_marker_image, markers)
        assert output.shape == single_marker_image.shape

    def test_modifies_pixels_when_markers_present(
        self, calibrated_detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        markers = calibrated_detector.detect_markers(single_marker_image)
        output = calibrated_detector.draw_detections(single_marker_image, markers)
        # Drawing should change at least some pixels
        assert not np.array_equal(output, single_marker_image)

    def test_handles_empty_markers(
        self, detector: ArucoDetection, single_marker_image: np.ndarray
    ) -> None:
        output = detector.draw_detections(single_marker_image, [])
        # Should return an unmodified copy
        np.testing.assert_array_equal(output, single_marker_image)


# ── draw_frame ───────────────────────────────────────────────────────────


class TestDrawFrame:
    def test_draws_bounding_box(
        self, detector: ArucoDetection, two_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(two_marker_image)
        frame = detector.compute_frame(markers)
        output = detector.draw_frame(two_marker_image, frame)
        assert output.shape == two_marker_image.shape
        assert not np.array_equal(output, two_marker_image)

    def test_returns_copy(
        self, detector: ArucoDetection, two_marker_image: np.ndarray
    ) -> None:
        markers = detector.detect_markers(two_marker_image)
        frame = detector.compute_frame(markers)
        output = detector.draw_frame(two_marker_image, frame)
        assert output is not two_marker_image
