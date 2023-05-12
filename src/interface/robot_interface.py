#!/usr/bin/env python3

# General interface for robot funcitonality

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

robotPublishers = dict()

class RobotInterface(Node):
    #
    # Constructors/Destructors
    #

    # def __init__(self):
    #     super().__init__('robot_interface_publisher')
    #     self.publisher_ = self.create_publisher(float, 'topic', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0

    def __init__(self, rosNode):
        self._rosNode = rosNode
        print("GOT TO __init__")

        # Drive base
        publisher = self._rosNode.create_publisher(String, 'front_left_drive_motor_velocity_from_interface', 10)
        robotPublishers['front_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'front_right_drive_motor_velocity_from_interface', 10)
        robotPublishers['front_right_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'mid_left_drive_motor_velocity_from_interface', 10)
        robotPublishers['mid_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'mid_right_drive_motor_velocity_from_interface', 10)
        robotPublishers['mid_right_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rear_left_drive_motor_velocity_from_interface', 10)
        robotPublishers['rear_left_drive_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rear_right_drive_motor_velocity_from_interface', 10)
        robotPublishers['rear_right_drive_motor'] = publisher
        
        # Arm
        publisher = self._rosNode.create_publisher(String, 'arm_turntable_motor_position_from_interface', 10)
        robotPublishers['arm_turntable_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_shoulder_motor_position_from_interface', 10)
        robotPublishers['arm_shoulder_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_elbow_motor_position_from_interface', 10)
        robotPublishers['arm_elbow_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_forearm_motor_position_from_interface', 10)
        robotPublishers['arm_forearm_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_wrist_motor_position_from_interface', 10)
        robotPublishers['arm_wrist_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_hand_motor_position_from_interface', 10)
        robotPublishers['arm_hand_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm_fingers_motor_position_from_interface', 10)
        robotPublishers['arm_fingers_motor'] = publisher

        # Antenna
        publisher = self._rosNode.create_publisher(String, 'antenna_motor_position_from_interface', 10)
        robotPublishers['antenna_motor'] = publisher
        publisher = self._rosNode.create_publisher(String, 'antenna_turntable_motor_position_from_interface', 10)
        robotPublishers['antenna_turntable_motor'] = publisher

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1
    
    def __del__(self):
        print()

    
    # TESTING
    #
    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1
    

    # RobotInterface(const RobotInterface&) = delete;
    # RobotInterface& operator=(const RobotInterface&) = delete;
    # RobotInterface(RobotInterface&&) = delete;
    # RobotInterface& operator=(RobotInterface&&) = delete;


    #
    # Movement
    #

    # General movement including all wheels
    def moveForward():
        print()

    def moveBackward(self, amount):
        print()
    def turnLeft(self, amount):
        print()
    def turnRight(self, amount):
        print()

    # Left front wheel
    def front_left_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['front_left_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing front_left_drive_motor: "%s"' % strMsg.data)

    # Right front wheel
    def front_right_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['front_right_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing front_right_drive_motor: "%s"' % strMsg.data)

    # Left middle wheel
    def mid_left_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['mid_left_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing mid_left_drive_motor: "%s"' % strMsg.data)

    # Right middle wheel
    def mid_right_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['mid_right_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing mid_right_drive_motor: "%s"' % strMsg.data)

    # Left back wheel
    def rear_left_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['rear_left_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing rear_left_drive_motor: "%s"' % strMsg.data)

    # Right back wheel
    def rear_right_drive_motor(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['rear_right_drive_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing rear_right_drive_motor: "%s"' % strMsg.data)

    # Arm turntable
    def arm_turntable_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_turntable_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_turntable_motor: "%s"' % strMsg.data)

    # Arm Shoulder movement
    def arm_shoulder_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_shoulder_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_shoulder_motor: "%s"' % strMsg.data)

    # Arm Elbow movement
    def arm_elbow_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_elbow_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_elbow_motor: "%s"' % strMsg.data)

    # Arm Forearm movement
    def arm_forearm_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_forearm_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_forearm_motor: "%s"' % strMsg.data)
    
    # Arm Forearm movement
    def arm_wrist_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_wrist_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_wrist_motor: "%s"' % strMsg.data)

    # Arm Hand movement
    def arm_hand_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_hand_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_hand_motor: "%s"' % strMsg.data)
    
    # Arm Finger movement
    def arm_fingers_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm_fingers_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm_fingers_motor: "%s"' % strMsg.data)

    # Antenna movement
    def antenna_motor_extend(self):
        # Output is in "revolutions"
        revolutionsOutput = 1 # we don't know the actual position it should be. this is a placeholder

        publisher = robotPublishers['antenna_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antenna_motor: "%s"' % strMsg.data)

    def antenna_motor_retract(self):
        # Output is in "revolutions"
        revolutionsOutput = 0 # we don't know the actual position it should be. this is a placeholder

        publisher = robotPublishers['antenna_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antenna_motor: "%s"' % strMsg.data)

    def antenna_turntable_motor(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['antenna_turntable_motor']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antenna_turntable_motor: "%s"' % strMsg.data)


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

