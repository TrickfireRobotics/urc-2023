import json
import std_msgs.msg
from std_msgs.msg import String
from threading import Thread
from threading import Lock
import moteus

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from utility.moteus_data_in_json_helper import MoteusDataInJsonHelper
from utility.moteus_data_out_json_helper import MoteusDataOutJsonHelper

class MoteusMotor():

    def __init__(self, canID, name, rosNode):
        self.canID = canID
        self.name = name
        self.hasDataEverBeenSent = False
        self._rosNode = rosNode

        self._subscriber = self._createSubscriber()
        self._publisher = self._createPublisher()
        
        #Mutex
        self.mutex_lock = Lock()


        # The settings we can send to the moteus controller
        # using the make_position() method
        self.position = None
        self.velocity = None
        self.feedfoward_torque = None
        self.kp_scale = None
        self.kd_scale = None
        self.max_torque = None
        self.watchdog_timeout = None
        self.velocity_limit = None
        self.accel_limit = None
        self.fixed_votlage_override = None
        self.ilimit_scale = None
        
        self.setStop = True

    
    def _createSubscriber(self):
        topicName = self.name + "_from_interface"
        subscriber = self._rosNode.create_subscription(
            std_msgs.msg.String, 
            topicName, 
            self.dataInCallback,
            1
        )
        
        return subscriber


    def _createPublisher(self):
        topicName = self.name + "_from_can"
        publisher = self._rosNode.create_publisher(
            std_msgs.msg.String,
            topicName,
            1
        )

        return publisher


    def dataInCallback(self, msg):
        self.mutex_lock.acquire()
        try:
            jsonString = msg.data
            self.updateMotorState(jsonString)
        finally:
            self.mutex_lock.release()
        


    def updateMotorState(self, rawJSONString):
        jsonHelper = MoteusDataInJsonHelper()
        jsonHelper.buildHelper(rawJSONString)
        
        self.position = jsonHelper.position
        self.velocity = jsonHelper.position
        self.feedfoward_torque = jsonHelper.feedfoward_torque
        self.kp_scale = jsonHelper.kp_scale
        self.kd_scale = jsonHelper.kd_scale
        self.max_torque = jsonHelper.max_torque
        self.watchdog_timeout = jsonHelper.watchdog_timeout
        self.accel_limit = jsonHelper.accel_limit
        self.fixed_votlage_override = jsonHelper.fixed_votlage_override
        self.ilimit_scale = jsonHelper.ilimit_scale
        self.setStop = jsonHelper.setStop


    def publishData(self, moteusData):
        self.mutex_lock.acquire()
        try:
            jsonHelper = MoteusDataOutJsonHelper()
            jsonHelper.position = moteusData[moteus.Register.POSITION]
            jsonHelper.velocity = moteusData[moteus.Register.VELOCITY]
            jsonHelper.torque = moteusData[moteus.Register.TORQUE]
            jsonHelper.temperature = moteusData[moteus.Register.TEMPERATURE]
            jsonHelper.power = moteusData[moteus.Register.POWER]
            jsonHelper.qCurrent = moteusData[moteus.Register.Q_CURRENT]
            jsonHelper.dCurrent = moteusData[moteus.Register.D_CURRENT]
            jsonHelper.inputVoltage = moteusData[moteus.Register.VOLTAGE]
            jsonHelper.voltagePhaseA = moteusData[moteus.Register.VOLTAGE_PHASE_A]
            jsonHelper.voltagePhaseB = moteusData[moteus.Register.VOLTAGE_PHASE_B]
            jsonHelper.voltagePhaseC = moteusData[moteus.Register.VOLTAGE_PHASE_C]
            
            jsonString = jsonHelper.buildJSONString()
            
            msg = String()
            msg.data = jsonString
            
            self._publisher.publish(msg)
            
        finally:
            self.mutex_lock.release()

