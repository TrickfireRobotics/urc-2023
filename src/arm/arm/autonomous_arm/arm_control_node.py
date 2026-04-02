import rclpy
import spatialmath as sm
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.interface.robot_interface import RobotInterface
from lib.interface.robot_info import RobotInfo
from arm.inverse_kinematics import InverseKinematics

MOVE_TIMEOUT_SEC = 3.0  # seconds to wait after commanding arm before publishing "done"


class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_node")

        self._interface = RobotInterface(self)
        self._info = RobotInfo(self)
        self._ik = InverseKinematics(self, self._interface, self._info)
        self._move_timer = None

        # ============ Subscribers ============
        self.pose_subscriber = self.create_subscription(Pose, "arm/pose", self.pose_callback, 1)

        # ============ Publishers ============
        self.status_publisher = self.create_publisher(String, "arm/typing/status", 1)

    # ============ Callbacks ============
    def pose_callback(self, msg: Pose) -> None:
        self.get_logger().info(f"Received new arm pose to move to: {msg}")
        if self._move_timer is not None:
            self._move_timer.cancel()
            self._move_timer = None
        self.move_arm_to_pose(msg)
        self._move_timer = self.create_timer(MOVE_TIMEOUT_SEC, self._on_move_complete)

    def _on_move_complete(self) -> None:
        if self._move_timer is not None:
            self._move_timer.cancel()
            self._move_timer = None
        self._ik.can_send = False
        self.publish_status("done")

    # ============ Main Logic ============
    def move_arm_to_pose(self, pose: Pose) -> None:
        target_se3 = self._pose_to_se3(pose)
        self._ik.last_target = sm.SE3(self._ik.target)
        self._ik.target = target_se3
        self._ik.can_send = True
        self._ik.runArmToTarget()

    @staticmethod
    def _pose_to_se3(pose: Pose) -> sm.SE3:
        q = pose.orientation
        # geometry_msgs uses scalar-last (x,y,z,w); spatialmath UnitQuaternion uses scalar-first (w,x,y,z)
        unit_quat = sm.UnitQuaternion([q.w, q.x, q.y, q.z], norm=True)
        rotation = unit_quat.SO3()
        translation = [pose.position.x, pose.position.y, pose.position.z]
        return sm.SE3.Rt(rotation, translation)

    def publish_status(self, status: str) -> None:
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Published arm typing status: {status}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    arm_control_node = None
    try:
        arm_control_node = ArmControlNode()
        rclpy.spin(arm_control_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if arm_control_node is not None:
            arm_control_node.get_logger().info("Shutting down arm_control_node")
    finally:
        if arm_control_node is not None:
            arm_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
