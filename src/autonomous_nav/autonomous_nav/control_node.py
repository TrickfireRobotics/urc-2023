"""
This node is the middleman between the decision_making_node and the drivebase.
Subscribes to wheel velocity commands from decision_making_node and republishes to drivebase.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.color_codes import ColorCodes, colorStr


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        # Velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Subscribers
        self.create_subscription(Float32, "/left_wheel_velocity", self.left_wheel_callback, 10)
        self.create_subscription(Float32, "/right_wheel_velocity", self.right_wheel_callback, 10)

        # Publishers
        self.left_wheel_pub = self.create_publisher(
            Float32, "/move_left_drivebase_side_message", 10
        )
        self.right_wheel_pub = self.create_publisher(
            Float32, "/move_right_drivebase_side_message", 10
        )

    def left_wheel_callback(self, msg: Float32) -> None:
        self.left_wheel_velocity = msg.data
        self.publish_left_wheel_commands()

    def right_wheel_callback(self, msg: Float32) -> None:
        self.right_wheel_velocity = msg.data
        self.publish_right_wheel_commands()

    def publish_right_wheel_commands(self) -> None:
        right_msg = Float32()
        right_msg.data = self.right_wheel_velocity
        self.right_wheel_pub.publish(right_msg)

    def publish_left_wheel_commands(self) -> None:
        left_msg = Float32()
        left_msg.data = self.left_wheel_velocity
        self.left_wheel_pub.publish(left_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    control_node = None
    try:
        control_node = ControlNode()
        rclpy.spin(control_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if control_node is not None:
            control_node.get_logger().info(
                colorStr("Shutting down control_node", ColorCodes.BLUE_OK)
            )
    finally:
        if control_node is not None:
            control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
