#!/usr/bin/env python3

# General interface for robot funcitonality

import rclpy
from rclpy.node import Node
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32
import math

VELOCITY_CONVERSION = 1
POSITION_CONVERSION = 1 / (6.28) #Radians to position ticks
REV_TO_RADIANS = 6.28 # One Revolution = 2pi

from utility.moteus_data_in_json_helper import MoteusDataInJsonHelper

robotPublishers = dict()

class RobotInterface(Node):
    #
    # Constructors/Destructors
    #
    
    def __init__(self, rosNode):
        self._rosNode = rosNode

        # Drive base
        publisher = self._rosNode.create_publisher(String, 'front_left_drive_motor_from_interface', 10)
        robotPublishers['front_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'front_right_drive_motor_from_interface', 10)
        robotPublishers['front_right_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'mid_left_drive_motor_from_interface', 10)
        robotPublishers['mid_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'mid_right_drive_motor_from_interface', 10)
        robotPublishers['mid_right_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rear_left_drive_motor_from_interface', 10)
        robotPublishers['rear_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rear_right_drive_motor_from_interface', 10)
        robotPublishers['rear_right_drive_motor'] = publisher
        
        robotPublishers['arm_fingers_motor'] = publisher

        # Antenna
        publisher = self._rosNode.create_publisher(Float32, 'antenna_motor_position_from_interface', 10)
        robotPublishers['antenna_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'antenna_turntable_motor_position_from_interface', 10)
        robotPublishers['antenna_turntable_motor'] = publisher
        
        
        # Arm
        publisher = self._rosNode.create_publisher(String, 'arm_turntable_motor_from_interface', 10)
        robotPublishers['arm_turntable_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_shoulder_motor_from_interface', 10)
        robotPublishers['arm_shoulder_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_elbow_motor_from_interface', 10)
        robotPublishers['arm_elbow_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_left_wrist_motor_from_interface', 10)
        robotPublishers['arm_left_wrist_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_right_wrist_motor_from_interface', 10)
        robotPublishers['arm_right_wrist_motor'] = publisher
        
    
    def velocityConversion(self, amount):
        return amount * VELOCITY_CONVERSION

    # Converting from input "radians" to output "revolutions"
    def positionConversion(self, amount):
        return amount * POSITION_CONVERSION
    
    # General movement including all wheels
    def moveForward():
        print()
    def moveBackward(self, amount):
        print()
    def turnLeft(self, amount):
        print()
    def turnRight(self, amount):
        print()

    # ------ DRIVEBASE ------
    # Left front wheel - MEASURED IN REVOLUTIONS PER SECOND
    def front_left_drive_motor(self, amount):        
        self._send_drive_motor("front_left_drive_motor", radiansPerSecond = (amount))

    # Right front wheel
    def front_right_drive_motor(self, amount):
        self._send_drive_motor("front_right_drive_motor", radiansPerSecond = (amount))

    # Left middle wheel
    def mid_left_drive_motor(self, amount):
        self._send_drive_motor("mid_left_drive_motor", radiansPerSecond = (amount))

    # Right middle wheel
    def mid_right_drive_motor(self, amount):
        self._send_drive_motor("mid_right_drive_motor", radiansPerSecond = (amount))

    # Left back wheel
    def rear_left_drive_motor(self, amount):
        self._send_drive_motor("rear_left_drive_motor", radiansPerSecond = (amount))

    # Right back wheel
    def rear_right_drive_motor(self, amount):
        self._send_drive_motor("rear_right_drive_motor", radiansPerSecond = (amount))

    def _send_drive_motor(self, pubName, radiansPerSecond = 0):
        publisher = robotPublishers[pubName]
        
        strMsg = String()
        jsonHelper = MoteusDataInJsonHelper()
        jsonHelper.velocity = radiansPerSecond
        jsonHelper.setStop = False
        
        strMsg.data = jsonHelper.buildJSONString()
        publisher.publish(strMsg)

    # ------ ARM ------
    
    def arm_turntable_velocity(self, targetVel):
        self._send_arm_velocity("arm_turntable_motor", targetVel)
    
    def shoulder_velocity(self, targetVel):
        self._send_arm_velocity("arm_shoulder_motor", targetVel)
    
    def elbow_velocity(self, targetVel):
        self._send_arm_velocity("arm_elbow_motor", targetVel)
    
    def leftWrist_velocity(self, targetVel):
        self._send_arm_velocity("arm_left_wrist_motor", targetVel)
    
    def rightWrist_velocity(self, targetVel):
        self._send_arm_velocity("arm_right_wrist_motor", targetVel)

    # For now, I am sending rev/sec
    def _send_arm_velocity(self, pubName, velocity = 0):
        publisher = robotPublishers[pubName]
        
        strMsg = String()
        jsonHelper = MoteusDataInJsonHelper()
        jsonHelper.velocity = velocity
        jsonHelper.setStop = False
        
        strMsg.data = jsonHelper.buildJSONString()
        publisher.publish(strMsg)
        
        
    def disable_arm_turntable_motor(self):
        self._disable_motor("arm_turntable_motor")
    
    def disable_arm_shoulder_motor(self):
        self._disable_motor("arm_shoulder_motor")
    
    def disable_arm_elbow_motor(self):
        self._disable_motor("arm_elbow_motor")
    
    def disable_arm_left_wrist_motor(self):
        self._disable_motor("arm_left_wrist_motor")
    
    def disable_arm_right_wrist_motor(self):
        self._disable_motor("arm_right_wrist_motor")
    
    def _disable_motor(self, pubName):
        publisher = robotPublishers[pubName]
        
        strMsg = String()
        jsonHelper = MoteusDataInJsonHelper()
        jsonHelper.velocity = 0 # for extra security
        jsonHelper.setStop = True
        
        strMsg.data = jsonHelper.buildJSONString()
        publisher.publish(strMsg)
        
        

    # Antenna movement
    def antenna_motor_extend(self):
        # Output is in "revolutions"
        revolutionsOutput = 1 # we don't know the actual position it should be. this is a placeholder
        publisher = robotPublishers['antenna_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    def antenna_motor_retract(self):
        # Output is in "revolutions"
        revolutionsOutput = 0 # we don't know the actual position it should be. this is a placeholder
        publisher = robotPublishers['antenna_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    def antenna_turntable_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount)
        publisher = robotPublishers['antenna_turntable_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)
