import myactuator_rmd_py as rmd
import rclpy
import std_msgs.msg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8MotorState, RMDX8RunSettings

from .rmdx8_motor import RMDx8Motor


class RMDx8MotorManager(Node):
    """
    Docsting
    """

    def __init__(self, ros_node: Node) -> None:
        super().__init__("can_rmdx8_node")
        self.get_logger().info(colorStr("Launching can_rmdx8 node", ColorCodes.BLUE_OK))
        self._id_to_rmdx8_motor: dict[int, RMDx8Motor]
        self._ros_node = ros_node

    def addMotor(self, config: RMDx8MotorConfig) -> None:
        """
        Adds new rmdx8 motor to the motor dictionary
        """

        # Create a motor
        motor = RMDx8Motor(config, self._ros_node)
        self._id_to_rmdx8_motor[config.can_id] = motor

    def create_motors(self) -> None:
        """
        Create all necessary RMDx8 motors and add them to the dictionary
        """

        # self.addMotor( some_config)
        # self.addMotor( some_config)
        # self.addMotor( some_config)
        # self.addMotor( some_config)
        # self.addMotor( some_config)
        # self.addMotor( some_config)
        # self.addMotor( some_config)

    # Main function
    def main(self, args: list[str] | None = None) -> None:
        """
        The entry point for RMDx8
        """
