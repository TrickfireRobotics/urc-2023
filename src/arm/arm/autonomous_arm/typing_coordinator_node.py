"""
This node replaces the message_handler_node.py

Receives a string msg from Mission Control being the message that needs to be typed
Sends out task to arm_control_node.py that contains the character that is to be pressed on the keyboard
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class TypingCoordinatorNode(Node):
    def __init__(self) -> None:
        self.message = ""
        self.char_index = 0  # tracks progress of typing

        # ============ Subscribers ============
        self.typing_message_subscribtion = self.create_subscription(
            String, "arm/typing/message", self.message_callback, 1
        )

        self.arm_done_subscribtion = self.create_subscription(
            String, "arm/typing/status", self.arm_status_callback, 1
        )

        # ============ Publishers ============
        self.next_char_pub = self.create_publisher(String, "arm/typing/next_char", 1)

    # ============ Callbacks ============
    def message_callback(self, msg: String) -> None:
        self.get_logger().info(f"Received typing mission message: {msg.data}")
        self.message = msg.data

    def arm_status_callback(self, msg: String) -> None:
        self.get_logger().info(f"Arm finished character: {msg.data}")
        self.char_index += 1
        self.publish_next_char()

    # ============ Main Logic ============

    def publish_next_char(self) -> None:
        if self.char_index >= len(self.message):
            self.get_logger().info("Typing complete")
            return
        char_msg = String()
        char_msg.data = self.message[self.char_index]
        self.next_char_pub.publish(char_msg)
        self.get_logger().info(
            f"Published char: '{char_msg.data}' ({self.char_index}/{len(self.message)})"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    typing_coordinator_node = None
    try:
        typing_coordinator_node = TypingCoordinatorNode()
        rclpy.spin(typing_coordinator_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if typing_coordinator_node is not None:
            typing_coordinator_node.get_logger().info("Shutting down typing_coordinator_node")
    finally:
        if typing_coordinator_node is not None:
            typing_coordinator_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
