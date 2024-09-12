import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.color_codes import ColorCodes
from lib.interface.robot_interface import RobotInterface


class Drivebase(Node):
    SPEED = 1.5

    def __init__(self) -> None:
        super().__init__("drivebase")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching drivebase node" + ColorCodes.ENDC)

        self.bot_interface = RobotInterface(self)

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10
        )
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10
        )

    def moveLeftSide(self, msg: Float32) -> None:
        vel = msg.data * self.SPEED
        self.bot_interface.frontLeftDriveMotor(-vel)
        self.bot_interface.midLeftDriveMotor(-vel)
        self.bot_interface.rearLeftDriveMotor(-vel)

    def moveRightSide(self, msg: Float32) -> None:
        vel = msg.data * self.SPEED
        self.bot_interface.frontRightDriverMotor(vel)
        self.bot_interface.midRightDriveMotor(vel)
        self.bot_interface.rearRightDriveMotor(vel)

    def turnLeft(self, msg: Float32) -> None:
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg: Float32) -> None:
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)


def main(args: list[str] | None = None) -> None:
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
        drivebase.get_logger().info(
            ColorCodes.BLUE_OK + "Shutting down drivebase" + ColorCodes.ENDC
        )
        drivebase.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
