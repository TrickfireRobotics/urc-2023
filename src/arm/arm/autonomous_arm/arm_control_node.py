import rclpy
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from pymoveit2 import MoveIt2

JOINT_NAMES = ["shoulder_1", "elbow_1", "wrist_1", "wrist_2"]
HOME_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0]


class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_node")

        callback_group = ReentrantCallbackGroup()
        self.moveit_client = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name="base_link",
            end_effector_name="gripper_assembly",
            group_name="arm",
            callback_group=callback_group,
        )
        self.moveit_client.max_velocity = 0.1
        self.moveit_client.max_acceleration = 0.1

        # ============ Subscribers ============
        self.pose_subscriber = self.create_subscription(Pose, "arm/pose", self.pose_callback, 1)

        # ============ Publishers ============
        self.status_publisher = self.create_publisher(String, "arm/typing/status", 1)

    # ============ Callbacks ============
    def pose_callback(self, msg: Pose) -> None:
        """Move arm to the target key pose, then return to home."""
        self.get_logger().info(f"Received new arm pose to move to: {msg}")

        # Move to the key pose
        self.move_arm_to_pose(msg)
        success = self.moveit_client.wait_until_executed()

        if success:
            self.publish_status("done")
            # Return to home position
            self.moveit_client.move_to_configuration(joint_positions=HOME_JOINT_POSITIONS)
            self.moveit_client.wait_until_executed()
            self.publish_status("ready")
        else:
            self.get_logger().error("Failed to move arm to target pose")
            self.publish_status("error")

    # ============ Main Logic ============
    def move_arm_to_pose(self, pose: Pose) -> bool:
        if pose is None:
            self.get_logger().error("Received None pose, cannot move arm")
            return False

        # self.moveit_client.move_to_pose(
        #     pose=pose,
        #     frame_id="base_link",
        #     tolerance_position=0.001,
        #     tolerance_orientation=0.001,
        #     cartesian=False,
        # )

        self.moveit_client.plan(
            pose=pose,
            frame_id="base_link",
            tolerance_position=0.001,
            tolerance_orientation=0.001,
            cartesian=False,
        )

        self.moveit_client.execute()

        self.moveit_client.wait_until_executed()
        return True

    def publish_status(self, status: str) -> None:
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f"Published arm typing status: {status}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    arm_control_node = ArmControlNode()
    # Multi-threaded executor is required for this node to run with the pymoveit2 action server
    # Pymoveit2 runs rcply.spin once internally
    executor = MultiThreadedExecutor(2)
    executor.add_node(arm_control_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        arm_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
