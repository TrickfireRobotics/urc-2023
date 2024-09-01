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
    "front_right_drive_motor_from_can"
    "mid_right_drive_motor_from_can",
    "rear_right_drive_motor_from_can",
    "arm_turntable_motor_from_can",
    "arm_shoulder_motor_from_can",
    "arm_elbow_motor_from_can",
    "arm_left_wrist_motor_from_can",
    "arm_right_wrist_motor_from_can"
}

canIDToMotorName = {
    CanBusMappings.CANID_FRONT_LEFT_DRIVE_MOTOR: "front_left_drive_motor_from_can",
    CanBusMappings.CANID_MID_LEFT_DRIVE_MOTOR: "mid_left_drive_motor_from_can",
    CanBusMappings.CANID_REAR_LEFT_DRIVE_MOTOR: "rear_left_drive_motor_from_can",
    CanBusMappings.CANID_FRONT_RIGHT_DRIVE_MOTOR: "front_right_drive_motor_from_can",
    CanBusMappings.CANID_MID_RIGHT_DRIVE_MOTOR: "mid_right_drive_motor_from_can",
    CanBusMappings.CANID_REAR_RIGHT_DRIVE_MOTOR: "rear_right_drive_motor_from_can",
    CanBusMappings.CANID_ARM_TURNTABLE_MOTOR: "arm_turntable_motor",
    CanBusMappings.CANID_ARM_SHOULDER_MOTOR: "arm_shoulder_motor",
    CanBusMappings.CANID_ARM_ELBOW_MOTOR: "arm_elbow_motor",
    CanBusMappings.CANID_ARM_LEFT_WRIST_MOTOR: "arm_left_wrist_motor",
    CanBusMappings.CANID_ARM_RIGHT_WRIST_MOTOR: "arm_right_wrist_motor"
}



class RobotInfo():
    
    
    def __init__(self, rosNode : Node):
        self._rosNode = rosNode
        self.subList = [] # empty array
        self.canIDToJSON = {} #Dict
        self.createSubscribers()
        
        self.canIDToJSON[20] = MoteusDataOutJsonHelper()
        self.canIDToJSON[21] = MoteusDataOutJsonHelper()
        self.canIDToJSON[22] = MoteusDataOutJsonHelper()
        self.canIDToJSON[23] = MoteusDataOutJsonHelper()
        self.canIDToJSON[24] = MoteusDataOutJsonHelper()
        self.canIDToJSON[25] = MoteusDataOutJsonHelper()
        
        # self.moteusNameToJSON = {}
        # self.subList = []
        # self.createNameToJSONMapping()
        # self.createSubscribers()
        
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
        
    # def createNameToJSONMapping(self):
    #     for name in moteusTopicList:
    #         self.moteusNameToJSON[name] = MoteusDataOutJsonHelper()
            
    # def createSubscribers(self):
    #     for name in moteusTopicList:
    #         sub = self._rosNode.create_subscription(String, name, self.subCallback, 1)
    #         self.subList.append(sub)
    
    # def subCallback(self, msg):
    #     jsonHelper = MoteusDataOutJsonHelper()
    #     jsonHelper.buildHelper(msg.data)
    #     moteusName = canIDToMotorName[jsonHelper.canID]
    #     self.moteusNameToJSON[moteusName] = jsonHelper
        
        
        
    # def getMoteusMotorData(self, canID):
    #     topicName = canIDToMotorName[canID]        
    #     return self.moteusNameToJSON[topicName]
        
        
        