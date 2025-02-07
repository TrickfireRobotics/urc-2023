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

        # setting these based on the assumption that when we initialize this class, the arm
        # is in its default "rest"
        self.motorStartingAngles = [
            0.0,
            57.3,
            8.113,
            # 0,
            # 0,
        ]

        self.setArmOffsets()

        # calculated using the l + r wrist positions
        self.wrist_angle = 0.0
        self.wrist_rot_angle = 0.0

        # TODO INITIALIZE IK STUFF HERE -------------
        # Import robot urdf from the resources folder
        current_dir = os.path.dirname(__file__)
        urdf_file_path = os.path.join(current_dir, "..", "resource", "arm.urdf")
        urdf_file_path = os.path.normpath(urdf_file_path)

        # Initialise model
        self.viator = ERobot.URDF(urdf_file_path)

        # Elementary transforms, basically rotation and translations in xyz
        self.ets = self.viator.ets()

        # #########################################################################################
        # Set goal pose
        # Tep is basically for storing coords & rotation
        # Use spatial math sm to add the xyz & roll pitch yaw relative to the position of the hand
        # (.q), which we got using forward kinematics (fkine)

        # TODO these values should be initialized calling forward kinematics with the current joint
        # angles
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self._ros_node.get_logger().info(
            "Target position:", self.target_x, self.target_y, self.target_z
        )

        self.Tep = (
            self.viator.fkine(self.viator.q)
            * sm.SE3(self.target_x, self.target_y, self.target_z)
            * sm.SE3.RPY([0, 0, 0], order="xyz")
            * sm.SE3.Rz(90, unit="deg")
        )

        self.jointDict: dict[str, tuple] = {}

        # Make our solver
        self.solver = rtb.IK_LM()

        self.up_x_sub = ros_node.create_subscription(Float32, "shoulder_up", self.xUp, 10)

        self.down_x_sub = ros_node.create_subscription(Float32, "shoulder_down", self.xDown, 10)

        self.up_z_sub = ros_node.create_subscription(Float32, "elbow_up", self.xUp, 10)

        self.down_z_sub = ros_node.create_subscription(Float32, "elbow_down", self.xDown, 10)

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

    def solve(self) -> None:
        self._ros_node.get_logger().info(
            "Target position:", self.target_x, self.target_y, self.target_z
        )
        self.Tep = (
            self.viator.fkine(self.viator.q)
            * sm.SE3(self.target_x, self.target_y, self.target_z)
            * sm.SE3.RPY([0, 0, 0], order="xyz")
            * sm.SE3.Rz(90, unit="deg")
        )

        # Solve the IK problem
        self.solver.solve(self.ets, self.Tep)

        for i, link in enumerate(self.viator.links):
            joint_name = link.name if link.name else f"Joint {i}"
            pose = self.viator.fkine(self.viator.q, end=joint_name)  # FK up to joint i

            pos = pose.t  # Translation (x, y, z)
            rpy = pose.rpy(order="xyz", unit="deg")  # Rotation in roll-pitch-yaw (degrees)

            self.jointDict[joint_name] = (pos, rpy)
            self._ros_node.get_logger().info(
                f"│ {i:^4} │ {joint_name:^9} │ SE3({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}; "
                f"{rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°) │"
            )

    def xUp(self, msg: Float32) -> None:
        self.target_x += 0.5
        self.solve()

    def xDown(self, msg: Float32) -> None:
        self.target_x -= 0.5
        self.solve()

    def zUp(self, msg: Float32) -> None:
        self.target_z += 0.5
        self.solve()

    def zDown(self, msg: Float32) -> None:
        self.target_x -= 0.5
        self.solve()
