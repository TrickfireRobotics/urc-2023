import math
import os
from enum import IntEnum

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
from rclpy.node import Node
from roboticstoolbox import ERobot
from std_msgs.msg import Float32

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


class ArmMotorsEnum(IntEnum):
    TURNTABLE = 0
    SHOULDER = 1
    ELBOW = 2
    LEFTWRIST = 3
    RIGHTWRIST = 4


class InverseKinematics:

    # constants
    REVS_TO_RADIANS = math.pi * 2.0
    DEGREES_TO_RADIANS = math.pi / 180.0
    RADIANS_TO_DEGREES = 1.0 / DEGREES_TO_RADIANS

    def __init__(self, ros_node: Node, interface: RobotInterface, info: RobotInfo):

        self._ros_node = ros_node
        self._interface = interface
        self._info = info

        self.motorConfigList = [
            MotorConfigs.ARM_TURNTABLE_MOTOR,
            MotorConfigs.ARM_SHOULDER_MOTOR,
            MotorConfigs.ARM_ELBOW_MOTOR,
            # MotorConfigs.ARM_LEFT_WRIST_MOTOR,
            # MotorConfigs.ARM_RIGHT_WRIST_MOTOR,
        ]

        self.motorOffsetList = [
            0.0,
            0.0,
            0.0,
            # 0.0,
            # 0.0
        ]
        self.setArmOffsets()

        # setting these based on the assumption that when we initialize this class, the arm
        # is in its default "rest"
        self.motorStartingAngles = [
            0,
            0,
            0,
            # 0,
            # 0,
        ]

        # calculated using the l + r wrist positions
        self.wrist_angle = 0.0
        self.wrist_rot_angle = 0.0

        # TODO INITIALIZE IK STUFF HERE -------------
        # Import robot urdf from the resources folder
        current_dir = os.path.dirname(__file__)
        urdf_file_path = os.path.join(current_dir, "..", "resource", "arm.urdf")
        urdf_file_path = os.path.normpath(urdf_file_path)

        # Initialise model
        viator = ERobot.URDF(urdf_file_path)

        # TODO these values should be initialized calling forward kinematics with the current joint angles
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self._ros_node.get_logger().info(
            "Target position:", self.target_x, self.target_y, self.target_z
        )

    def setArmOffsets(self) -> None:
        for motorConfig in range(len(self.motorConfigList)):
            self.motorOffsetList[motorConfig] = self.motorStartingAngles[
                motorConfig
            ] - self.getMeasuredMotorAngle(motorConfig)

    def getMeasuredMotorAngle(self, motor: int) -> float:
        """
        Returns the angle of the motor IN DEGREES based on int representing motor config's index
        in the list of motor configs
        """
        _position = self._info.getMotorState(self.motorConfigList[motor]).position
        position_degrees = (
            (_position if _position is not None else 0.0)
            * self.REVS_TO_RADIANS
            * self.RADIANS_TO_DEGREES
        )
        return position_degrees

    def getPerceivedMotorAngle(self, motor: int) -> float:
        """
        Returns what the angle of the arm is in our IK representation of the arm
        """
        measuredAngle = self.getMeasuredMotorAngle(motor)
        return measuredAngle + self.motorOffsetList[motor]

    def runMotorPosition(self, motor: int, targetDegrees: float) -> None:
        targetRadians = targetDegrees * self.DEGREES_TO_RADIANS
        motorConfig = self.motorConfigList[motor]
        self._interface.runMotorPosition(motorConfig, targetRadians)

    def runAllMotorsToPosition(self, targetDegreeList: list) -> None:
        for motorConfig in range(len(targetDegreeList)):
            self.runMotorPosition(motorConfig, targetDegreeList[motorConfig])
