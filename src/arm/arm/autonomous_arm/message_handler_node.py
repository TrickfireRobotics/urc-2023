# this node reads the message we want to type on the keybaord and sends the location of that key on the keybaord to the arm control node
'''
This node needs
-an stdin reader to type in the message
-a subscriber to the keyboard input message
-a publisher for the caracter to be typed

this node needs to know the pose and orientation of the keyboard in the robot base frame
using that it will calculate the position of the key to be pressed

def cauculate_key_position(key: str, keyboard_pose: Pose) -> Pose:
    """
    Calculate the position of the key on the keyboard given the keyboard pose in the robot base frame
    """
def collect_keyboard_input() -> str:
    """
    Collect keyboard input from stdin
-
'''
import json
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from lib.color_codes import ColorCodes, colorStr


class MessageHandlerNode(Node):
    def __init__(self) -> None:
        super().__init__("message_handler_node")
        self.key_positions = None
        with open("src/arm/arm/autonomous_arm/key_positions.json") as f:
            self.key_positions = json.load(f)

        # ===== PUBLISHERS =====

        # Sends one character at a time to the motion planner
        self.key_publisher = self.create_publisher(String, "/key_to_press", 10)

        # Broadcasts typing status for the C2 station display
        self.status_publisher = self.create_publisher(String, "/typing_status", 10)

        # ===== SUBSCRIBERS =====

        # Receives success/fail from the motion planner after each keypress
        self.create_subscription(Bool, "/key_press_result", self.key_press_callback, 10)

        # ===== STATE VARIABLES =====

        self.user_message: Optional[str] = None
        self.message_index: int = 0          # which character we're currently on
        self.status: str = "Not Started"     # current lifecycle state
        self.waiting_for_result: bool = False  # True while motion planner is pressing a key
        self.is_backspacing: bool = False      # True when we sent BACKSPACE to recover from a failed keypress

        # Start input thread
        input_thread = threading.Thread(target=self.collect_user_input)
        input_thread.daemon = True
        input_thread.start()

    def update_status(self, status: str) -> None:
        """Update the typing state and publish it to /typing_status."""
        self.status = status
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Typing status: {status}")

    def collect_user_input(self) -> None:
        """Collect user input in a separate thread"""
        try:
            self.user_message = input("Enter the message to type (3-6 characters): ")
            self.get_logger().info(f"Received message: {self.user_message}")
            self.update_status("In Progress")
            self.send_next_character()
        except EOFError:
            self.get_logger().info("Input stream closed")

    def send_next_character(self) -> None:
        """Publish the current character to /key_to_press and wait for result."""
        if self.user_message is None:
            return
        key = self.user_message[self.message_index].upper()
        self.get_logger().info(f"Sending key: '{key}' ({self.message_index + 1}/{len(self.user_message)})")
        msg = String()
        msg.data = key
        self.key_publisher.publish(msg)
        self.waiting_for_result = True

    def key_press_callback(self, msg: Bool) -> None:
        """Called when the motion planner reports success or failure on /key_press_result."""
        if not self.waiting_for_result:
            return
        self.waiting_for_result = False

        if msg.data:
            if self.is_backspacing:
                # Backspace succeeded — retry the same character
                self.is_backspacing = False
                self.send_next_character()
            else:
                # Character typed successfully — move to next
                self.message_index += 1
                if self.user_message is None or self.message_index >= len(self.user_message):
                    self.update_status("Complete")
                else:
                    self.send_next_character()
        else:
            # Failure — send BACKSPACE to undo, then retry same character
            self.get_logger().warning("Keypress failed, sending BACKSPACE and retrying")
            self.is_backspacing = True
            backspace_msg = String()
            backspace_msg.data = "BACKSPACE"
            self.key_publisher.publish(backspace_msg)
            self.waiting_for_result = True

    def calculate_key_position(self, key: str) -> tuple[float, float, float] | None:
        """
        Calculate the position of the key on the keyboard given the keyboard pose in the robot base frame
        UNFINSHED
        """
        if self.key_positions and key in self.key_positions:
            return self.key_positions[key]
        else:
            self.get_logger().warning(f"Key {key} not found in key positions.")
            return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    message_handler_node = None

    try:
        message_handler_node = MessageHandlerNode()
        rclpy.spin(message_handler_node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        if message_handler_node is not None:
            message_handler_node.get_logger().info(
                colorStr("Shutting down message_handler_node", ColorCodes.BLUE_OK)
            )
    finally:
        if message_handler_node is not None:
            message_handler_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
