import math
from typing import Any, Callable, get_args, Literal, TYPE_CHECKING
import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.canbus_mappings import CanBusMappings
from utility.moteus_data_out_json_helper import MoteusDataOutJsonHelper

from rclpy.node import Node
from std_msgs.msg import String



moteusTopicList = {
    "front_left_drive_motor_from_can",
    "mid_left_drive_motor_from_can",
    "rear_left_drive_motor_from_can",
    "front_right_drive_motor_from_can",
    "mid_right_drive_motor_from_can",
    "rear_right_drive_motor_from_can",
    "arm_turntable_motor_from_can",
    "arm_shoulder_motor_from_can",
    "arm_elbow_motor_from_can",
    "arm_left_wrist_motor_from_can",
    "arm_right_wrist_motor_from_can",
}



class RobotInfo():
    
    
    def __init__(self, rosNode : Node):
        self._rosNode = rosNode
        self.subList = [] # empty array
        self.canIDToJSON = {} #Dict
        
        self.canIDToJSON[20] = MoteusDataOutJsonHelper()
        self.canIDToJSON[21] = MoteusDataOutJsonHelper()
        self.canIDToJSON[22] = MoteusDataOutJsonHelper()
        self.canIDToJSON[23] = MoteusDataOutJsonHelper()
        self.canIDToJSON[24] = MoteusDataOutJsonHelper()
        self.canIDToJSON[25] = MoteusDataOutJsonHelper()
        self.canIDToJSON[1] = MoteusDataOutJsonHelper()
        self.canIDToJSON[2] = MoteusDataOutJsonHelper()
        self.canIDToJSON[3] = MoteusDataOutJsonHelper()
        self.canIDToJSON[4] = MoteusDataOutJsonHelper()
        self.canIDToJSON[5] = MoteusDataOutJsonHelper()
        
        self.createSubscribers()
        
    def createSubscribers(self):
        for topicName in moteusTopicList:
            sub = self._rosNode.create_subscription(String, topicName, self.subCallback, 1)
            self.subList.append(sub)
            
    def subCallback(self, msg):
        jsonHelper = MoteusDataOutJsonHelper()
        jsonHelper.buildHelper(msg.data)
        self.canIDToJSON[jsonHelper.canID] = jsonHelper
        
    def getDataFromCanID(self, canID):          
        return self.canIDToJSON[canID]

        
        