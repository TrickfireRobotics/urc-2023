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
        publisher = self._rosNode.create_publisher(String, 'leftFrontWheel_Velocity_from_interface', 10)
        robotPublishers['leftFrontWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rightFrontWheel_Velocity_from_interface', 10)
        robotPublishers['rightFrontWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'leftMiddleWheel_Velocity_from_interface', 10)
        robotPublishers['leftMiddleWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rightMiddleWheel_Velocity_from_interface', 10)
        robotPublishers['rightMiddleWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'leftBackWheel_Velocity_from_interface', 10)
        robotPublishers['leftBackWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'rightBackWheel_Velocity_from_interface', 10)
        robotPublishers['rightBackWheel'] = publisher
        publisher = self._rosNode.create_publisher(String, 'baseRotation_Position_from_interface', 10)
        robotPublishers['baseRotation'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm1_Position_from_interface', 10)
        robotPublishers['arm1'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm2_Position_from_interface', 10)
        robotPublishers['arm2'] = publisher
        publisher = self._rosNode.create_publisher(String, 'arm3_Position_from_interface', 10)
        robotPublishers['arm3'] = publisher
        publisher = self._rosNode.create_publisher(String, 'antenna_Position_from_interface', 10)
        robotPublishers['antenna'] = publisher
        publisher = self._rosNode.create_publisher(String, 'antennaTurntable_Position_from_interface', 10)
        robotPublishers['antennaTurntable'] = publisher

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
    def leftFrontWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['leftFrontWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing leftFrontWheel: "%s"' % strMsg.data)

    # Right front wheel
    def rightFrontWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['rightFrontWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing rightFrontWheel: "%s"' % strMsg.data)

    # Left middle wheel
    def leftMiddleWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['leftMiddleWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing leftMiddleWheel: "%s"' % strMsg.data)

    # Right middle wheel
    def rightMiddleWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['rightMiddleWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing rightMiddleWheel: "%s"' % strMsg.data)

    # Left back wheel
    def leftBackWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['leftBackWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing leftBackWheel: "%s"' % strMsg.data)

    # Right back wheel
    def rightBackWheel(self, amount):
        # Converting from input "meters per second" to output "revolutions per second"
        revolutionsOutput = 1.76838724851 * amount

        publisher = robotPublishers['rightBackWheel']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing rightBackWheel: "%s"' % strMsg.data)

    #
    # Arm movement/rotation
    #

    # Base turntable
    def baseRotation(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['baseRotation']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing baseRotation: "%s"' % strMsg.data)

    # Arm 1 is closest to base and arm 3 is furthest from base

    # Arm 1 movement
    def arm1(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm1']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm1: "%s"' % strMsg.data)

    # Arm 2 movement
    def arm2(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm2']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm2: "%s"' % strMsg.data)

    # Arm 3 movement
    def arm3(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['arm3']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing arm3: "%s"' % strMsg.data)

    # Atenna movement
    def antennaExtend(self):
        # Output is in "revolutions"
        revolutionsOutput = 1 # we don't know the actual position it should be. this is a placeholder

        publisher = robotPublishers['antenna']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antenna: "%s"' % strMsg.data)

    def antennaRetract(self):
        # Output is in "revolutions"
        revolutionsOutput = 0 # we don't know the actual position it should be. this is a placeholder

        publisher = robotPublishers['antenna']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antenna: "%s"' % strMsg.data)

    def antennaTurntable(self, amount):
        # Converting from input "degrees" to output "revolutions"
        revolutionsOutput = amount / 360

        publisher = robotPublishers['antennaTurntable']
        strMsg = String()
        strMsg.data = str(revolutionsOutput)
        publisher.publish(strMsg)

        print('Publishing antennaTurntable: "%s"' % strMsg.data)


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

    object1.leftFrontWheel(10)
    object1.rightFrontWheel(15)
    object1.leftMiddleWheel(20)
    object1.rightMiddleWheel(25)
    object1.leftBackWheel(30)
    object1.rightBackWheel(9999)
    
    rclpy.spin(testNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    testNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

