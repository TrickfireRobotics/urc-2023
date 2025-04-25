import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr


class MotherboardDriver(Node):

    def __init__(self) -> None:
        super().__init__("mbd_node")
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.BLUE_OK))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        mbd = MotherboardDriver()
        rclpy.spin(mbd)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        mbd.get_logger().info(colorStr("Shutting down example_node node", ColorCodes.BLUE_OK))
        mbd.destroy_node()
        sys.exit(0)
