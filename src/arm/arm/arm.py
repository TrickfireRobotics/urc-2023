import sys
from enum import IntEnum
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32

from custom_interfaces.srv import ArmMode
from lib.color_codes import ColorCodes
from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface

from .individual_control_vel import IndividualControlVel


class ArmModeEnum(IntEnum):
    DISABLED = 0
    INDIVIDUAL_MOTOR_CONTROL_VEL = 1
    INDIVIDUAL_MOTOR_CONTROL_POS = 2
    INVERSE_KINEMATICS = 3


class Arm(Node):
    def __init__(self) -> None:
        super().__init__("arm_node")
        self.get_logger().info(ColorCodes.BLUE_OK + "Launching arm_node" + ColorCodes.ENDC)

        self.change_arm_mode_sub = self.create_subscription(
            Int32, "update_arm_mode", self.updateArmMode, 10
        )

        self.current_mode = ArmModeEnum.DISABLED

        self.mode_service = self.create_service(ArmMode, "get_arm_mode", self.modeServiceHandler)

        self.bot_interface = RobotInterface(self)

        self.individual_control_vel = IndividualControlVel(self, self.bot_interface)

    def modeServiceHandler(self, _: Any, response: ArmMode.Response) -> ArmMode.Response:
        response.current_mode = int(self.current_mode)
        return response

    def updateArmMode(self, msg: Int32) -> None:
        self.current_mode = msg.data

        if self.current_mode == 0:
            self.bot_interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)
            self.bot_interface.stopMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

            self.individual_control_vel.can_send = False
        elif self.current_mode == 1:
            self.individual_control_vel.can_send = True
        elif self.current_mode == 2:
            self.individual_control_vel.can_send = False
        elif self.current_mode == 3:
            self.individual_control_vel.can_send = False


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        node = Arm()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(ColorCodes.BLUE_OK + "Shutting down arm_node node" + ColorCodes.ENDC)
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
