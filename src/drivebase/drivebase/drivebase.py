import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface


class Drivebase(Node):
    SPEED = 6.28 * 1.5

    def __init__(self) -> None:
        super().__init__("drivebase")
        self.get_logger().info(colorStr("Launching drivebase node", ColorCodes.BLUE_OK))
        self.bot_interface = RobotInterface(self)

        self.left_subscription = self.create_subscription(
            Float32, "move_left_drivebase_side_message", self.moveLeftSide, 10
        )
        self.right_subscription = self.create_subscription(
            Float32, "move_right_drivebase_side_message", self.moveRightSide, 10
        )

        self.create_timer(0.05, self.stopMotors)
        self._last_received: list[float] = []

    def appendToLastReceived(self) -> None:
        last_receieved_index = len(self._last_received) - 1
        self._last_received.append(time.time())

    def moveLeftSide(self, msg: Float32) -> None:
        self.appendToLastReceived()
        vel = msg.data * self.SPEED
        self.bot_interface.runMotorSpeed(MotorConfigs.FRONT_LEFT_DRIVE_MOTOR, -vel)
        self.bot_interface.runMotorSpeed(MotorConfigs.MID_LEFT_DRIVE_MOTOR, -vel)
        self.bot_interface.runMotorSpeed(MotorConfigs.REAR_LEFT_DRIVE_MOTOR, -vel)

    def moveRightSide(self, msg: Float32) -> None:
        self.appendToLastReceived()
        vel = msg.data * self.SPEED
        self.bot_interface.runMotorSpeed(MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR, vel)
        self.bot_interface.runMotorSpeed(MotorConfigs.MID_RIGHT_DRIVE_MOTOR, vel)
        self.bot_interface.runMotorSpeed(MotorConfigs.REAR_RIGHT_DRIVE_MOTOR, vel)

    def turnLeft(self, msg: Float32) -> None:
        self.appendToLastReceived()
        self.moveLeftSide(msg)
        self.moveRightSide(-msg)

    def turnRight(self, msg: Float32) -> None:
        self.appendToLastReceived()
        self.moveLeftSide(-msg)
        self.moveRightSide(msg)

    def stopMotors(self) -> None:
        if len(self._last_received) == 0:
            self._last_received.append(time.time())
        last_receieved_index = len(self._last_received) - 1
        time_delta = time.time() - self._last_received[last_receieved_index]
        if time_delta > 0.25:

            self.get_logger().info("STOPPING ALL MOTORS")
            self.bot_interface.stopMotor(MotorConfigs.FRONT_LEFT_DRIVE_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.MID_LEFT_DRIVE_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.REAR_LEFT_DRIVE_MOTOR)

            self.bot_interface.stopMotor(MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.MID_RIGHT_DRIVE_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.REAR_RIGHT_DRIVE_MOTOR)


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
        drivebase.get_logger().info(colorStr("Shutting down drivebase", ColorCodes.BLUE_OK))
        drivebase.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
