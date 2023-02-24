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
        publisher = self._rosNode.create_publisher(String, 'topic', 10)
        robotPublishers['leftFrontWheelForward'] = publisher
        publisher = self._rosNode.create_publisher(String, 'topic', 10)
        robotPublishers['leftFrontWheelBackward'] = publisher
        

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
    def leftFrontWheelForward(self, amount):
        publisher = robotPublishers['leftFrontWheelForward']
        strMsg = String()
        strMsg.data = str(amount)
        publisher.publish(strMsg)

        print('Publishing leftFrontWheelForward: "%s"' % strMsg.data)
    def leftFrontWheelBackward(self, amount):
        publisher = robotPublishers['leftFrontWheelBackward']
        strMsg = String()
        strMsg.data = str(amount)
        publisher.publish(strMsg)

        print('Publishing leftFrontWheelBackward: "%s"' % strMsg.data)

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


class MyTestingNode(Node):
    def __init__(self):
        super().__init__('my_testing_node')
        #self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Hello ROS2")
    #def timer_callback(self):
    #    self.get_logger().info("Hello ROS2")

def main(args=None):
    rclpy.init(args=args)

    node = MyTestingNode()

    object1 = RobotInterface(node)

    minimal_publisher = object1.leftFrontWheelForward(10)
    minimal_publisher = object1.leftFrontWheelBackward(999)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

