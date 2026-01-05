import glob
import os
import pickle
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

# Camera calibration parameters
# You can modify these variables as needed
CHESSBOARD_SIZE = (8, 5)  # Number of inner corners per chessboard row and column (for 9x6  squares)
SQUARE_SIZE = 2.5  # Size of a square in centimeters
CALIBRATION_IMAGES_PATH = "calibration_images/*.jpg"  # Path to calibration images
OUTPUT_DIRECTORY = "output"  # Directory to save calibration results
SAVE_UNDISTORTED = True  # Whether to save undistorted images


def calibrate_camera() -> Tuple[
    Optional[float],
    Optional[np.ndarray[Any, np.dtype[Any]]],
    Optional[np.ndarray[Any, np.dtype[Any]]],
    Optional[List[np.ndarray[Any, np.dtype[Any]]]],
    Optional[List[np.ndarray[Any, np.dtype[Any]]]],
]:
    """
    Calibrate the camera using chessboard images.

    Returns:
        ret: The RMS re-projection error
        mtx: Camera matrix
        dist: Distortion coefficients
        rvecs: Rotation vectors
        tvecs: Translation vectors
    """
    # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (8,5,0)
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : CHESSBOARD_SIZE[0], 0 : CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

    # Scale object points by square size (for real-world measurements)
    objp = objp * SQUARE_SIZE

    # Arrays to store object points and image points from all images
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Get list of calibration images
    images = glob.glob(CALIBRATION_IMAGES_PATH)

    if not images:
        print(f"No calibration images found at {CALIBRATION_IMAGES_PATH}")
        return None, None, None, None, None

    # Create output directory if it doesn't exist
    if not os.path.exists(OUTPUT_DIRECTORY):
        os.makedirs(OUTPUT_DIRECTORY)

    print(f"Found {len(images)} calibration images")

    # Process each calibration image
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHESSBOARD_SIZE,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
        )
        print(f"Found {(corners)} corners in image {fname}")

        # If found, add object points and image points
        if ret:
            objpoints.append(objp)

            # Refine corner positions
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)

            # Save image with corners drawn
            output_img_path = os.path.join(OUTPUT_DIRECTORY, f"corners_{os.path.basename(fname)}")
            cv2.imwrite(output_img_path, img)

            print(f"Processed image {idx+1}/{len(images)}: {fname} - Chessboard found")
        else:
            print(f"Processed image {idx+1}/{len(images)}: {fname} - Chessboard NOT found")

    if not objpoints:
        print("No chessboard patterns were detected in any images.")
        return None, None, None, None, None

    print("Calibrating camera...")

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Save calibration results
    calibration_data = {
        "camera_matrix": mtx,
        "distortion_coefficients": dist,
        "rotation_vectors": rvecs,
        "translation_vectors": tvecs,
        "reprojection_error": ret,
    }

    with open(os.path.join(OUTPUT_DIRECTORY, "calibration_data.pkl"), "wb") as f:
        pickle.dump(calibration_data, f)

    # Save camera matrix and distortion coefficients as text files
    np.savetxt(os.path.join(OUTPUT_DIRECTORY, "camera_matrix.txt"), mtx)
    np.savetxt(os.path.join(OUTPUT_DIRECTORY, "distortion_coefficients.txt"), dist)

    print(f"Calibration complete! RMS re-projection error: {ret}")
    print(f"Results saved to {OUTPUT_DIRECTORY}")

    return ret, mtx, dist, rvecs, tvecs


def undistort_images(
    mtx: np.ndarray[Any, np.dtype[Any]], dist: np.ndarray[Any, np.dtype[Any]]
) -> None:
    """
    Undistort all calibration images using the calibration results.

    Args:
        mtx: Camera matrix
        dist: Distortion coefficients
    """
    if not SAVE_UNDISTORTED:
        return

    images = glob.glob(CALIBRATION_IMAGES_PATH)

    if not images:
        print(f"No images found at {CALIBRATION_IMAGES_PATH}")
        return

    undistorted_dir = os.path.join(OUTPUT_DIRECTORY, "undistorted")
    if not os.path.exists(undistorted_dir):
        os.makedirs(undistorted_dir)

    print(f"Undistorting {len(images)} images...")

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        h, w = img.shape[:2]

        # Refine camera matrix based on free scaling parameter
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # Undistort image
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # Crop the image (optional)
        x, y, w, h = roi
        dst = dst[y : y + h, x : x + w]

        # Save undistorted image
        output_img_path = os.path.join(undistorted_dir, f"undistorted_{os.path.basename(fname)}")
        cv2.imwrite(output_img_path, dst)

        print(f"Undistorted image {idx+1}/{len(images)}: {fname}")

    print(f"Undistorted images saved to {undistorted_dir}")


def calculate_reprojection_error(
    objpoints: List[np.ndarray[Any, np.dtype[Any]]],
    imgpoints: List[np.ndarray[Any, np.dtype[Any]]],
    mtx: np.ndarray[Any, np.dtype[Any]],
    dist: np.ndarray[Any, np.dtype[Any]],
    rvecs: List[np.ndarray[Any, np.dtype[Any]]],
    tvecs: List[np.ndarray[Any, np.dtype[Any]]],
) -> float:
    """
    Calculate the reprojection error for each calibration image.

    Args:
        objpoints: 3D points in real world space
        imgpoints: 2D points in image plane
        mtx: Camera matrix
        dist: Distortion coefficients
        rvecs: Rotation vectors
        tvecs: Translation vectors

    Returns:
        mean_error: Mean reprojection error
    """
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
        print(f"Reprojection error for image {i+1}: {error}")

    mean_error = total_error / len(objpoints)
    print(f"Mean reprojection error: {mean_error}")

    return mean_error


def main() -> None:
    """
    Main function to run the camera calibration process.
    """
    print("Starting camera calibration...")

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = calibrate_camera()

    if mtx is None:
        print("Calibration failed. Exiting.")
        return

    # Undistort images
    undistort_images(mtx, dist)

    print("Camera calibration completed successfully!")


if __name__ == "__main__":
    main()
