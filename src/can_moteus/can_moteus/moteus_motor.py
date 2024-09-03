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
        """
            Create a logical representation of a motor that is using
            a Moteus controller. Contains a list of variables that should be
            sent to the Moteus controller. Subscribes to input and publishes output
            
            Paramaters
            -------
            canID : int
                The canID of the Moteus Controller
            name : string
                The name of the motor. This is used in the topic names
            rosNode : Node
                The ROS node used to create the ros_moteus_bridge.py
        """
        
        self.canID = canID
        self.name = name
        self._rosNode = rosNode

        self._subscriber = self._createSubscriber()
        self._publisher = self._createPublisher()
        
        #Mutex - used to protect writing/reading the state of the motor
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
        
        #By default, the motor is shutoff
        self.setStop = True

    
    def _createSubscriber(self):
        """
            The subscriber to get data from.
            The format of the topic is the following: <motor name>_from_interface
        """
        topicName = self.name + "_from_interface"
        subscriber = self._rosNode.create_subscription(
            std_msgs.msg.String, 
            topicName, 
            self.dataInCallback,
            1 # Size of queue is 1. All additional ones are dropped
        )
        
        return subscriber


    def _createPublisher(self):
        """
            The publisher to send data to.
            The format of the topic is the following: <motor name>_from_can
        """
        topicName = self.name + "_from_can"
        publisher = self._rosNode.create_publisher(
            std_msgs.msg.String,
            topicName,
            1 # Size of queue is 1. All additional ones are dropped
        )

        return publisher


    def dataInCallback(self, msg):
        """
            Update the motor state. Mutex protected,
            meaning that no one can go into any other "critical section"
            of code that also has a mutex protecting it. 
        """
        self.mutex_lock.acquire()
        try:
            jsonString = msg.data
            self.updateMotorState(jsonString)
        finally:
            self.mutex_lock.release()
        


    def updateMotorState(self, rawJSONString):
        """
            Update the motor's variables from
            the input JSON string
        """
        
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
        self.setStop = jsonHelper.setStop


    def publishData(self, moteusData):
        """
            Publishes the data from the moteus controller
        """
        self.mutex_lock.acquire()
        try:
            if self._rosNode.context.ok:
                jsonHelper = MoteusDataOutJsonHelper()
                jsonHelper.canID = self.canID
                jsonHelper.position = moteusData.values[moteus.Register.POSITION]
                jsonHelper.velocity = moteusData.values[moteus.Register.VELOCITY]
                jsonHelper.torque = moteusData.values[moteus.Register.TORQUE]
                jsonHelper.temperature = moteusData.values[moteus.Register.TEMPERATURE]
                jsonHelper.power = moteusData.values[moteus.Register.POWER]
                jsonHelper.inputVoltage = moteusData.values[moteus.Register.VOLTAGE]
                jsonHelper.qCurrent = moteusData.values[moteus.Register.Q_CURRENT]
                jsonHelper.dCurrent = moteusData.values[moteus.Register.D_CURRENT]
                
                #self._rosNode.get_logger().info(str(moteusData.values))
                
                # TODO: We need to update firmware to get POWER information (7/1/2024)
                # https://github.com/mjbots/moteus/releases
                # https://discord.com/channels/633996205759791104/722434939676786688/1252380387783610428
                #self._rosNode.get_logger().info(str(moteusData.values.keys()))
                
                jsonString = jsonHelper.buildJSONString()
                
                msg = String()
                msg.data = jsonString
                
                self._publisher.publish(msg)
        except Exception as error:
            # This is used to handle any errors in order to prevent the thread from dying
            # Specifically, when we crtl-c we want the motors to be set_stop(), but if this thread
            # crashes we cannot do that. So we catch any errors
            self._rosNode.get_logger().info("Failed to publish motor data")
            self._rosNode.get_logger().info(str(error))
        finally:
            self.mutex_lock.release()

