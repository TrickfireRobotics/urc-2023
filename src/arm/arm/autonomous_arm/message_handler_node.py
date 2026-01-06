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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.color_codes import ColorCodes, colorStr


class MessageHandlerNode(Node):
    def __init__(self) -> None:
        super().__init__("message_handler_node")
        self.key_positions = None
        with open("src/autonomous_nav/autonomous_arm/key_positions.json") as f:
            self.key_positions = json.load(f)

        # ... existing code ...
        self.user_message = None

        # Start input thread
        input_thread = threading.Thread(target=self.collect_user_input)
        input_thread.daemon = True
        input_thread.start()

    def collect_user_input(self) -> None:
        """Collect user input in a separate thread"""
        try:
            self.user_message = input("Enter the message to type (3-6 characters): ")
            self.get_logger().info(f"Received message: {self.user_message}")
        except EOFError:
            self.get_logger().info("Input stream closed")

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
