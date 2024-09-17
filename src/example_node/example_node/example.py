import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lib.color_codes import ColorCodes, colorStr


class ExampleNode(Node):
    
    def __init__(self) -> None:
        super().__init__("my_example_node")
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.BLUE_OK))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        example = ExampleNode()
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        example.get_logger().info(colorStr("Shutting down example_node node", ColorCodes.BLUE_OK))
        example.destroy_node()
        sys.exit(0)
        
        
if __name__ == "__main__":
    main()        