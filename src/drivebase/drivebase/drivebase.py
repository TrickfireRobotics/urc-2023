import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from interface.robot_interface import RobotInterface

from std_msgs.msg import String, Float32


class Drivebase(Node):

    global botInterface
    global SPEED
    SPEED = 1

    def __init__(self):
        super().__init__('drivebase')

        global botInterface
        botInterface = RobotInterface(self)

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10)
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10)

    def moveLeftSide(self, msg):
        global botInterface
        vel = msg * SPEED
        botInterface.leftFrontWheel(vel)
        botInterface.leftMiddleWheel(vel)
        botInterface.leftBackWheel(vel)

    def moveRightSide(self, msg):
        global botInterface
        vel = msg * SPEED
        botInterface.rightFrontWheel(vel)
        botInterface.rightMiddleWheel(vel)
        botInterface.rightBackWheel(vel)

    def turnLeft(self, msg):
        global botInterface
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg):
        global botInterface
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)


def main(args=None):
    rclpy.init(args=args)

    drivebase = Drivebase()

    print("drivebase main")

    while True:
        drivebase.turnRight(0.5)

    rclpy.spin(drivebase)  # prints callbacks

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drivebase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
