import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs


class TempLight(Node):

    def __init__(self) -> None:
        super().__init__("temp_light_node")
        self.get_logger().info(colorStr("Launching temp_light node", ColorCodes.BLUE_OK))


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        node = TempLight()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(colorStr("Shutting down temp_light", ColorCodes.BLUE_OK))
        if node.thread_manager is not None:
            node.thread_manager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
