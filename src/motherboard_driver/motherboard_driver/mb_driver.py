import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr


class MotherboardDriver(Node):

    def __init__(self) -> None:
        super().__init__("mbd_node")
        self.get_logger().info(colorStr("Launching mbd_node node", ColorCodes.BLUE_OK))

    def print_test(self) -> None:
        self.get_logger().info(colorStr("HELLO WORLD!", ColorCodes.BLUE_OK))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        mbd = MotherboardDriver()
        mbd.print_test()
        rclpy.spin(mbd)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        mbd.get_logger().info(colorStr("Shutting down mbd_node node", ColorCodes.BLUE_OK))
        mbd.destroy_node()
        sys.exit(0)
