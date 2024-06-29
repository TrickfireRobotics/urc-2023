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
        self.feedforward_torque = None
        self.kp_scale = None
        self.kd_scale = None
        self.max_torque = None
        self.watchdog_timeout = None
        self.velocity_limit = None
        self.accel_limit = None
        self.fixed_voltage_override = None
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
        
        self.position = jsonHelper.getPosition()
        self.velocity = jsonHelper.getVelocity()
        self.feedforward_torque = jsonHelper.feedforward_torque
        self.kp_scale = jsonHelper.kp_scale
        self.kd_scale = jsonHelper.kd_scale
        self.max_torque = jsonHelper.max_torque
        self.watchdog_timeout = jsonHelper.watchdog_timeout
        self.accel_limit = jsonHelper.accel_limit
        self.fixed_voltage_override = jsonHelper.fixed_voltage_override
        self.ilimit_scale = jsonHelper.ilimit_scale
        self.setStop = jsonHelper.setStop


    def publishData(self, moteusData):
        self.mutex_lock.acquire()
        try:
            if self._rosNode.context.ok:
                jsonHelper = MoteusDataOutJsonHelper()
                jsonHelper.position = moteusData.values[moteus.Register.POSITION]
                jsonHelper.velocity = moteusData.values[moteus.Register.VELOCITY]
                jsonHelper.torque = moteusData.values[moteus.Register.TORQUE]
                jsonHelper.temperature = moteusData.values[moteus.Register.TEMPERATURE]
                #jsonHelper.power = moteusData.values[moteus.Register.POWER]
                #jsonHelper.qCurrent = moteusData.values[moteus.Register.Q_CURRENT]
                #jsonHelper.dCurrent = moteusData.values[moteus.Register.D_CURRENT]
                jsonHelper.inputVoltage = moteusData.values[moteus.Register.VOLTAGE]
                #jsonHelper.voltagePhaseA = moteusData.values[moteus.Register.VOLTAGE_PHASE_A]
                #jsonHelper.voltagePhaseB = moteusData.values[moteus.Register.VOLTAGE_PHASE_B]
                #jsonHelper.voltagePhaseC = moteusData.values[moteus.Register.VOLTAGE_PHASE_C]
                
                jsonString = jsonHelper.buildJSONString()
                
                msg = String()
                msg.data = jsonString
                
                self._publisher.publish(msg)
        except Exception as error:
            self._rosNode.get_logger().info("Failed to publish motor data")
        finally:
            self.mutex_lock.release()

