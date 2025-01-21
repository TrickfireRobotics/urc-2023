import sys

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
    Class to manage the control and storage of RMDx8 motors in the ROS system.
    """

    def __init__(self) -> None:
        super().__init__("can_rmdx8_node")
        self.get_logger().info(colorStr("Launching can_rmdx8 node", ColorCodes.BLUE_OK))
        self._id_to_rmdx8_motor: dict[int, RMDx8Motor] = {}

    def addMotor(self, config: RMDx8MotorConfig) -> None:
        """
        Adds new rmdx8 motor to the motor dictionary
        """

        # Create a motor
        motor = RMDx8Motor(config, self)
        self._id_to_rmdx8_motor[config.can_id] = motor

    def shutdownMotors(self) -> None:
        """
        Shutdown all motors
        """

        for motor in self._id_to_rmdx8_motor.values():
            motor.shutdownMotor()

    def createRMDx8Motors(self) -> None:
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
def main(args: list[str] | None = None) -> None:
    """
    The entry point for RMDx8
    """

    rclpy.init(args=args)
    try:
        node = RMDx8MotorManager()
        rclpy.spin(node)  # Keep node active
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.shutdownMotors()
        rclpy.shutdown()


# If script is run directly, then create a RMDx8MotorManager object and run the main function
if __name__ == "__main__":
    main(sys.argv)
