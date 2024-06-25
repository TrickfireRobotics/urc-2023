import sys
import rclpy
from rclpy.node import Node
import moteus
import threading
from . import moteus_thread_manager
from rclpy.executors import ExternalShutdownException

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.color_text import ColorCodes
from utility.canbus_mappings import CanBusMappings

class RosMotuesBridge(Node):

    def __init__(self):
        super().__init__("can_moteus_node")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching can_moteus node" + ColorCodes.ENDC)

        self.threadManager = moteus_thread_manager.MoteusThreadManager(self)
        self.canbusMappings = CanBusMappings()
        
        self.createMoteusMotors()
        



    def createMoteusMotors(self):
        self.threadManager.addMotor(self.canbusMappings.CANID_REAR_RIGHT_DRIVE_MOTOR, "rear_right_drive_motor")
        self.threadManager.addMotor(self.canbusMappings.CANID_MID_RIGHT_DRIVE_MOTOR, "mid_right_drive_motor")
        self.threadManager.addMotor(self.canbusMappings.CANID_FRONT_RIGHT_DRIVE_MOTOR, "front_right_drive_motor")
        
        self.threadManager.addMotor(self.canbusMappings.CANID_REAR_LEFT_DRIVE_MOTOR, "rear_left_drive_motor")
        self.threadManager.addMotor(self.canbusMappings.CANID_MID_LEFT_DRIVE_MOTOR, "mid_left_drive_motor")
        self.threadManager.addMotor(self.canbusMappings.CANID_FRONT_LEFT_DRIVE_MOTOR, "front_left_drive_motor")

        self.threadManager.start()
        
        
        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RosMotuesBridge()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.get_logger().info(ColorCodes.BLUE_OK + "Shutting down can_moteus" + ColorCodes.ENDC)
        node.threadManager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)

if __name__ == "__main__":
    main()