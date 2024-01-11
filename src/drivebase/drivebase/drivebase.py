import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from interface.robot_interface import RobotInterface

from std_msgs.msg import String, Float32


class Drivebase(Node):

    def __init__(self):
        super().__init__('drivebase')

        self.botInterface = RobotInterface(self)
        self.SPEED = 1.0

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10)
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10)

    def moveLeftSide(self, msg):
        vel = msg.data # * self.SPEED
        self.botInterface.front_left_drive_motor(vel)
        self.botInterface.mid_left_drive_motor(vel)
        self.botInterface.rear_left_drive_motor(vel)

    def moveRightSide(self, msg):
        vel = msg.data # * self.SPEED
        self.botInterface.front_right_drive_motor(vel)
        self.botInterface.mid_right_drive_motor(vel)
        self.botInterface.rear_right_drive_motor(vel)

    def turnLeft(self, msg):
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg):
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)


def main(args=None):
    rclpy.init(args=args)

    drivebase = Drivebase()

    print("drivebase main")

    rclpy.spin(drivebase)  # prints callbacks

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drivebase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
