import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import sys
sys.path.append("/home/trickfire/urc-2023/src")
from utility.color_text import ColorCodes


class Arm(Node):
    def __init__(self):
        super().__init__("arm_node")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching arm_node" + ColorCodes.ENDC)
        
        
        
def main(args=None):
    """
        The entry point of the node.
    """
    
    rclpy.init(args=args)
    try:
        node = Arm()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(ColorCodes.BLUE_OK + "Shutting down arm_node node" + ColorCodes.ENDC)
        node.threadManager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)

if __name__ == "__main__":
    main()
    