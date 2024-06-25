#!/usr/bin/env python3

# General interface for robot funcitonality

import rclpy
from rclpy.node import Node
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32

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
        print("GOT TO __init__")

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
        
        # Arm
        publisher = self._rosNode.create_publisher(Float32, 'arm_turntable_motor_position_from_interface', 10)
        robotPublishers['arm_turntable_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_shoulder_motor_position_from_interface', 10)
        robotPublishers['arm_shoulder_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_elbow_motor_position_from_interface', 10)
        robotPublishers['arm_elbow_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_forearm_motor_position_from_interface', 10)
        robotPublishers['arm_forearm_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_wrist_motor_position_from_interface', 10)
        robotPublishers['arm_wrist_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_hand_motor_position_from_interface', 10)
        robotPublishers['arm_hand_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'arm_fingers_motor_position_from_interface', 10)
        robotPublishers['arm_fingers_motor'] = publisher

        # Antenna
        publisher = self._rosNode.create_publisher(Float32, 'antenna_motor_position_from_interface', 10)
        robotPublishers['antenna_motor'] = publisher
        publisher = self._rosNode.create_publisher(Float32, 'antenna_turntable_motor_position_from_interface', 10)
        robotPublishers['antenna_turntable_motor'] = publisher
    
    def __del__(self):
        print()
    
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

    # Left front wheel - MEASURED IN REVOLUTIONS PER SECOND
    def front_left_drive_motor(self, amount):        
        self._send_drive_motor("front_left_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    # Right front wheel
    def front_right_drive_motor(self, amount):
        self._send_drive_motor("front_right_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    # Left middle wheel
    def mid_left_drive_motor(self, amount):
        self._send_drive_motor("mid_left_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    # Right middle wheel
    def mid_right_drive_motor(self, amount):
        self._send_drive_motor("mid_right_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    # Left back wheel
    def rear_left_drive_motor(self, amount):
        self._send_drive_motor("rear_left_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    # Right back wheel
    def rear_right_drive_motor(self, amount):
        self._send_drive_motor("rear_right_drive_motor", radiansPerSecond = (amount * REV_TO_RADIANS))

    def _send_drive_motor(self, pubName, radiansPerSecond = 0):
        publisher = robotPublishers[pubName]
        
        strMsg = String()
        jsonHelper = MoteusDataInJsonHelper()
        jsonHelper.velocity = radiansPerSecond
        jsonHelper.setStop = False
        
        strMsg.data = jsonHelper.buildJSONString()
        publisher.publish(strMsg)



    # Arm turntable
    def arm_turntable_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) 
        publisher = robotPublishers['arm_turntable_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    # Arm Shoulder movement
    def arm_shoulder_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) * 3.979 
        publisher = robotPublishers['arm_shoulder_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    # Arm Elbow movement
    def arm_elbow_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) * 5.204 
        publisher = robotPublishers['arm_elbow_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    # Arm Forearm movement
    def arm_forearm_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) * 3.820 
        publisher = robotPublishers['arm_forearm_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)
    
    # Arm Up Down Wrist movement
    def arm_wrist_updown_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) * 3.820 
        publisher = robotPublishers['arm_wrist_updown_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    # Arm Wrist rotation movement
    def arm_wrist_rotation_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount) / 6.283
        publisher = robotPublishers['arm_wrist_rotation_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)

    # Arm Hand movement
    def arm_hand_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount)
        publisher = robotPublishers['arm_hand_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
        publisher.publish(strMsg)
    
    # Arm Finger movement
    def arm_fingers_motor(self, amount):
        revolutionsOutput = self.positionConversion(amount)
        publisher = robotPublishers['arm_fingers_motor']
        strMsg = Float32()
        strMsg.data = revolutionsOutput
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


class MyTestingNode(Node):
    def __init__(self):
        super().__init__('my_testing_node')
        #self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Hello ROS2")
    #def timer_callback(self):
    #    self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)

    testNode = MyTestingNode()

    object1 = RobotInterface(testNode)

    object1.front_left_drive_motor(10)
    object1.front_right_drive_motor(15)
    object1.mid_left_drive_motor(20)
    object1.mid_right_drive_motor(25)
    object1.rear_left_drive_motor(30)
    object1.rear_right_drive_motor(9999)
    
    rclpy.spin(testNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    testNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
