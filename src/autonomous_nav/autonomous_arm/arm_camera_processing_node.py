# This nodes job is to give the autonomous arm system the location of each keyboard key relative to the robot base frame
# It does this through the following process:
# Step 1: acquires raw camera image
# Step 2: collects Aruco marker positions relative to camera
# Step 3: localizes those points to the robot base fram
# Step 4: creates an x,y, and z slope (AKA 3d plane) based off point positions
# Step 5: calculates the positions of all keys based on this plane
import cv2
import numpy as np

# --- Settings ---
MARKER_LENGTH = 0.025  # meters (25 mm marker, change as needed)

# Load camera calibration (example values)
cameraMatrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)

distCoeffs = np.zeros((5,))  # if you have real calibration, put it here

# --- Load ArUco dictionary ---
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Load image (replace with camera frame)
frame = cv2.imread("aruco_test.png")

# Detect markers
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, rejected = detector.detectMarkers(frame)

if ids is not None:
    # Estimate pose
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, MARKER_LENGTH, cameraMatrix, distCoeffs
    )

    for i, marker_id in enumerate(ids):
        tvec = tvecs[i][0]  # translation vector
        rvec = rvecs[i][0]  # rotation vector

        print(f"\nMarker ID: {marker_id[0]}")
        print(
            f"Position (tvec) in camera frame: x={tvec[0]:.3f} m, y={tvec[1]:.3f} m, z={tvec[2]:.3f} m"
        )
        print(f"Rotation (rvec): {rvec}")

        # Optional: draw axis on the image
        cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.02)

    # Draw detected markers
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

else:
    print("No ArUco markers detected.")

# Show the result
cv2.imshow("Frame", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
# --------------------------------------------------------------------------
#   ROS 2 Node Main
# --------------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    sensor_processing_node = None
    try:
        arm_camera_processing_node = SensorProcessingNode()
        rclpy.spin(sensor_processing_node)
    except KeyboardInterrupt:
        if sensor_processing_node is not None:
            sensor_processing_node.get_logger().info(
                "Keyboard interrupt received. Shutting down..."
            )
    finally:
        if sensor_processing_node is not None:
            cv2.destroyAllWindows()
            sensor_processing_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
