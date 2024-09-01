import sys
sys.path.append("/home/trickfire/urc-2023/src")
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from enum import IntEnum
from custom_interfaces.srv import ArmMode
from std_msgs.msg import Int32
from .individual_control_vel import IndividualControlVel
from interface.robot_interface import RobotInterface



from utility.color_text import ColorCodes


class ArmModeEnum(IntEnum):
    disabled = 0
    individual_motor_control_vel = 1
    individual_motor_control_pos = 2
    inverse_kinematics = 3
    

class Arm(Node):
    def __init__(self):
        super().__init__("arm_node")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching arm_node" + ColorCodes.ENDC)
        
        self.changeArmModeSub = self.create_subscription(Int32, "update_arm_mode", self.updateArmMode, 10)
        
        self.current_mode = ArmModeEnum.disabled
        
        self.mode_service = self.create_service(ArmMode, "get_arm_mode", self.mode_serviceHandler)
        
        self.botInterface = RobotInterface(self)
        
        self.individualControlVel = IndividualControlVel(self, self.botInterface)
        
    def mode_serviceHandler(self, request, response):
        response.current_mode = int(self.current_mode)        
        return response
    
    def updateArmMode(self, msg):
        self.current_mode = msg.data
        
        
        
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
    