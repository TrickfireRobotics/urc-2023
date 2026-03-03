import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class ArmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_node")

        # ============ Subscribers ============
        self.pose_subscriber = self.create_subscription(Pose, "arm/pose", self.pose_callback, 1)

        # ============ Publishers ============
        self.status_publisher = self.create_publisher(String, "arm/typing/status", 1)

    # ============ Callbacks ============
    def pose_callback(self, msg: Pose) -> None:
        self.get_logger().info(f"Received new arm pose to move to: {msg}")
        self.move_arm_to_pose(msg)
        self.publish_status("done")

    def move_arm_to_pose(self, pose: Pose) -> None:
        # TODO Implement actual arm movement logic here
        pass


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
