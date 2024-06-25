import rclpy
from rclpy.node import Node
import time

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from interface.robot_interface import RobotInterface

from std_msgs.msg import String, Float32

from rclpy.executors import ExternalShutdownException

from utility.color_text import ColorCodes



class Drivebase(Node):

    def __init__(self):
        super().__init__('drivebase')
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching drivebase node" + ColorCodes.ENDC)

        self.botInterface = RobotInterface(self)
        self.SPEED = 0.5

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10)
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10)

    def moveLeftSide(self, msg):
        vel = msg.data * self.SPEED
        self.botInterface.front_left_drive_motor(-vel)
        self.botInterface.mid_left_drive_motor(-vel)
        self.botInterface.rear_left_drive_motor(-vel)

    def moveRightSide(self, msg):
        vel = msg.data * self.SPEED
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
    try:
        drivebase = Drivebase()
        rclpy.spin(drivebase)  # prints callbacks
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        drivebase.get_logger().info(ColorCodes.BLUE_OK + "Shutting down drivebase" + ColorCodes.ENDC)
        drivebase.destroy_node()
        sys.exit(0)


if __name__ == '__main__':
    main()
