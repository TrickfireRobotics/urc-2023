import math
from typing import Any, Callable, get_args, Literal, TYPE_CHECKING
import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.canbus_mappings import CanBusMappings
from utility.moteus_data_out_json_helper import MoteusDataOutJsonHelper

from rclpy.node import Node
from std_msgs.msg import String

canBusMappings = CanBusMappings()

moteusTopicList = {
    "front_left_drive_motor_from_can",
    "mid_left_drive_motor_from_can",
    "rear_left_drive_motor_from_can",
    "front_right_drive_motor_from_can"
    "mid_right_drive_motor_from_can",
    "rear_right_drive_motor_from_can",
}

canIDToMotorName = {
    canBusMappings.CANID_FRONT_LEFT_DRIVE_MOTOR: "front_left_drive_motor_from_can",
    canBusMappings.CANID_MID_LEFT_DRIVE_MOTOR: "mid_left_drive_motor_from_can",
    canBusMappings.CANID_REAR_LEFT_DRIVE_MOTOR: "rear_left_drive_motor_from_can",
    canBusMappings.CANID_FRONT_RIGHT_DRIVE_MOTOR: "front_right_drive_motor_from_can",
    canBusMappings.CANID_MID_RIGHT_DRIVE_MOTOR: "mid_right_drive_motor_from_can",
    canBusMappings.CANID_REAR_RIGHT_DRIVE_MOTOR: "rear_right_drive_motor_from_can"
}



class RobotInfo():
    
    
    def __init__(self, rosNode : Node):
        self._rosNode = rosNode
        self.moteusNameToJSON = {}
        self.subList = []
        self.createNameToJSONMapping()
        self.createSubscribers()
        
        
    def createNameToJSONMapping(self):
        for name in moteusTopicList:
            self.moteusNameToJSON[name] = MoteusDataOutJsonHelper()
            
    def createSubscribers(self):
        for name in moteusTopicList:
            sub = self._rosNode.create_subscription(String, name, self.subCallback, 1)
            self.subList.append(sub)
    
    def subCallback(self, msg):
        jsonHelper = MoteusDataOutJsonHelper()
        jsonHelper.buildHelper(msg.data)
        moteusName = canIDToMotorName[jsonHelper.canID]
        self.moteusNameToJSON[moteusName] = jsonHelper
        
        
        
    def getMoteusMotorData(self, canID):
        topicName = canIDToMotorName[canID]
        return self.moteusNameToJSON[topicName]
        
        
        