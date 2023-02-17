#!/usr/bin/env python3

# General interface for robot funcitonality

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

    robotPublishers = dict()

    def __init__(self, rosNode):
        self._rosNode = rosNode
        publisher = self._rosNode.create_publisher(String, 'topic', 10)
        self.robotPublishers['moveForward'] = publisher
        

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
        publisher = super().robotPublisher['moveForward']
        str = String()
        str.data = "HELLO WORLD HELLO WORLD"
        publisher.publish(str)

    def moveBackward(self, amount):
        print()
    def turnLeft(self, amount):
        print()
    def turnRight(self, amount):
        print()

    # Left front wheel
    def leftFrontWheelForward(self, amount):
        print()
    def leftFrontWheelBackward(self, amount):
        print()

    # Right front wheel
    def rightFrontWheelForward(self, amount):
        print()
    def rightFrontWheelBackward(self, amount):
        print()

    # Left middle wheel
    def leftMiddleWheelForward(self, amount):
        print()
    def leftMiddleWheelBackward(self, amount):
        print()

    # Right middle wheel
    def rightMiddleWheelForward(self, amount):
        print()
    def rightMiddleWheelBackward(self, amount):
        print()

    # Left back wheel
    def leftBackWheelForward(self, amount):
        print()
    def leftBackWheelBackward(self, amount):
        print()

    # Right back wheel
    def rightBackWheelForward(self, amount):
        print()
    def rightBackWheelBackward(self, amount):
        print()

    #
    # Arm movement/rotation
    #

    # Base turntable
    def baseRotationLeft(self, amount):
        print()
    def baseRotationRight(self, amount):
        print()

    # Arm 1 is closest to base and arm 3 is furthest from base

    # Arm 1 movement
    def arm1Extend(self, amount):
        print()
    def arm1Retract(self, amount):
        print()

    # Arm 2 movement
    def arm2Extend(self, amount):
        print()
    def arm2Retract(self, amount):
        print()

    # Arm 3 movement
    def arm3Extend(self, amount):
        print()
    def arm3Retract(self, amount):
        print()

    # Atenna movement
    def antennaExtend(self):
        print()
    def antennaRetract(self):
        print()
    def antennaTurnLeft(self, amount):
        print()
    def antennaTurnRight(self, amount):
        print()


def main(args=None):
    rclpy.init(args=args)

    object1 = RobotInterface()

    minimal_publisher = object1.moveForward()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
