import sys

import myactuator_rmd_py as rmd
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

import lib.configs as Configs
from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs

from . import rmdx8_motor


class RMDx8Node(Node):
    """
    Our testing node.
    """

    def __init__(self) -> None:
        super().__init__("can_rmdx8_node")
        self.get_logger().info(colorStr("Launching can_rmdx8 node", ColorCodes.BLUE_OK))
        driver = rmd.CanDriver("can1")
        self.example_pub = self.create_publisher(String, "example_rmdx8_topic", 10)


def main(args: list[str] | None = None) -> None:
    """
    Entry point.
    """
    rclpy.init(args=args)
    try:
        example = RMDx8Node()
        # driver is an object from myactuator
        driver = rmd.CanDriver("can1")
        configs = MotorConfigs.RMDx8_TESTING_MOTOR
        motor = rmdx8_motor.RMDx8Motor(configs)
        motor._createPublisher()

        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        example.get_logger().info(colorStr("Shutting down can_rmdx8 node", ColorCodes.BLUE_OK))
        example.destroy_node()
        sys.exit(0)
