import myactuator_rmd_py as rmd
import rclpy
import std_msgs.msg
from rclpy.executors import ExternalShutdownException

# from moteus.moteus import Result
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8MotorState, RMDX8RunSettings


class RMDx8MotorManager(Node):
    """
    Docsting
    """

    def __init__(self) -> None:
        super().__init__("can_rmdx8_node")
        self.get_logger().info(colorStr("Launching can_rmdx8 node", ColorCodes.BLUE_OK))

    # Main function
    def main(self, args: list[str] | None = None) -> None:
        """
        The entry point of the node.
        """
