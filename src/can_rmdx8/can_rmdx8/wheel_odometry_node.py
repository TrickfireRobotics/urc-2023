"""
Subscribes to the 6 RMDx8 drive motor state topics, computes differential-drive
odometry and publishes nav_msgs/Odometry on /wheel_odom for the EKF to fuse.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.configs import MotorConfigs
from lib.motor_state.rmd_motor_state import RMDX8MotorState

# Physical constants
WHEEL_RADIUS = 0.11  # metres
WHEEL_BASE = 0.5  # metres (distance between left and right wheel contact patches)

# rev/s → m/s:  v = ω * r,   ω = 2π * (rev/s)
REV_PER_S_TO_M_PER_S = 2.0 * math.pi * WHEEL_RADIUS

# CAN IDs for left and right drive motors
LEFT_CAN_IDS = {
    MotorConfigs.FRONT_LEFT_DRIVE_MOTOR.can_id,
    MotorConfigs.MID_LEFT_DRIVE_MOTOR.can_id,
    MotorConfigs.REAR_LEFT_DRIVE_MOTOR.can_id,
}
RIGHT_CAN_IDS = {
    MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR.can_id,
    MotorConfigs.MID_RIGHT_DRIVE_MOTOR.can_id,
    MotorConfigs.REAR_RIGHT_DRIVE_MOTOR.can_id,
}


class WheelOdometryNode(Node):
    """
    Integrates wheel velocities from the RMDx8 drive motors into a pose estimate
    and publishes it as nav_msgs/Odometry on /wheel_odom.
    """

    def __init__(self) -> None:
        super().__init__("wheel_odometry_node")

        # Latest velocity reading (m/s) for each side, keyed by CAN ID
        self._left_vels: dict[int, float] = {}
        self._right_vels: dict[int, float] = {}

        # Integrated pose
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0  # yaw in radians

        self._last_time = self.get_clock().now()

        # Subscribe to all 6 drive motor state topics
        all_drive_configs = [
            MotorConfigs.FRONT_LEFT_DRIVE_MOTOR,
            MotorConfigs.MID_LEFT_DRIVE_MOTOR,
            MotorConfigs.REAR_LEFT_DRIVE_MOTOR,
            MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR,
            MotorConfigs.MID_RIGHT_DRIVE_MOTOR,
            MotorConfigs.REAR_RIGHT_DRIVE_MOTOR,
        ]
        for config in all_drive_configs:
            self.create_subscription(
                String,
                config.getCanTopicName(),
                self._make_motor_callback(config.can_id),
                10,
            )

        self._odom_pub = self.create_publisher(Odometry, "/wheel_odom", 10)

        # Integrate and publish at 20 Hz
        self.create_timer(0.05, self._publish_odometry)

        self.get_logger().info("WheelOdometryNode started, publishing to /wheel_odom")

    def _make_motor_callback(self, can_id: int):
        def callback(msg: String) -> None:
            state = RMDX8MotorState.fromJsonMsg(msg)
            if state.velocity is None or not math.isfinite(state.velocity):
                return
            # state.velocity is in rev/s — convert to m/s
            vel_m_s = state.velocity * REV_PER_S_TO_M_PER_S
            if can_id in LEFT_CAN_IDS:
                self._left_vels[can_id] = vel_m_s
            elif can_id in RIGHT_CAN_IDS:
                self._right_vels[can_id] = vel_m_s

        return callback

    def _publish_odometry(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0:
            return

        # Need at least one reading per side before publishing
        if not self._left_vels or not self._right_vels:
            return

        left_vel = sum(self._left_vels.values()) / len(self._left_vels)
        right_vel = sum(self._right_vels.values()) / len(self._right_vels)

        # Differential drive kinematics
        linear_vel = (right_vel + left_vel) / 2.0
        angular_vel = (right_vel - left_vel) / WHEEL_BASE

        # Euler integration
        self._x += linear_vel * math.cos(self._theta) * dt
        self._y += linear_vel * math.sin(self._theta) * dt
        self._theta += angular_vel * dt

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        # Yaw → quaternion (only rotation around z)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)

        # Twist
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel

        # Covariances (diagonal, moderate uncertainty for wheel odometry)
        # Order: x, y, z, roll, pitch, yaw
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05  # x
        pose_cov[7] = 0.05  # y
        pose_cov[35] = 0.1  # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01  # linear x
        twist_cov[35] = 0.05  # angular z
        odom.twist.covariance = twist_cov

        self._odom_pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
