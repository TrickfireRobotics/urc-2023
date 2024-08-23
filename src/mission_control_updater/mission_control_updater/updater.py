import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import sys
sys.path.append("/home/trickfire/urc-2023/src")

from . import infoToJSONHelper
from interface.robot_info import RobotInfo

from utility.color_text import ColorCodes

class MissionControlUpdater(Node):
    
    def __init__(self):
        super().__init__("mission_control_updater_node")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching mission_control_updater_node" + ColorCodes.ENDC)
        
        self.publisherToMC = self.create_publisher(
            String,
            "mission_control_updater",
            1
        )
        
        self.timer = self.create_timer(0.01, self.sendData)
        self.robotInfo = RobotInfo(self)
        
        
    def sendData(self):
        setOfCANID = {20,21,22,23,24,25}
        
        jsonBuilder = infoToJSONHelper.InfoToJSONHelper()
        
        for id in setOfCANID:
            jsonBuilder.addMoteusEntry(self.robotInfo.getDataFromCanID(id))
            
            
        msg = String()
        msg.data = jsonBuilder.buildJSONString()
        self.publisherToMC.publish(msg)
        
def main(args=None):
    """
        The entry point of the node.
    """
    
    rclpy.init(args=args)
    try:
        node = MissionControlUpdater()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(ColorCodes.BLUE_OK + "Shutting down mission_control_updater_node" + ColorCodes.ENDC)
        node.threadManager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)

if __name__ == "__main__":
    main()