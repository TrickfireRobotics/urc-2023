import math

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


class InverseKinematics:

    REVS_TO_RADIANS = math.pi * 2.0

    def __init__(self, ros_node: Node, interface: RobotInterface, info: RobotInfo):
        self._ros_node = ros_node
        self._interface = interface
        self._info = info

        # variables for arm joint angles
        # everything in radians
        self.shoulder_angle = 0.0
        self.elbow_angle = 0.0
        self.turntable_angle = 0.0
        self.l_wrist_angle = 0.0
        self.r_wrist_angle = 0.0

        # calculated using the l + r wrist positions
        self.wrist_angle = 0.0
        self.wrist_rot_angle = 0.0

        self.shoulder_offset = 0.0
        self.elbow_offset = 0.0
        self.turntable_offset = 0.0
        self.l_wrist_offset = 0.0
        self.r_wrist_offset = 0.0

        self.setArmJointOffsets()
        self.getArmAngles()

        arm_chain = Chain.from_urdf_file(
            "arm.urdf", active_links_mask=[True, True, True, True, True, True, True, True]
        )

    def setArmJointOffsets(self) -> None:
        self.shoulder_offset = self.shoulder_angle - self.getMotorPosition(
            MotorConfigs.ARM_SHOULDER_MOTOR
        )

        self.elbow_offset = self.elbow_angle - self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR)

        self.turntable_offset = self.turntable_angle - self.getMotorPosition(
            MotorConfigs.ARM_TURNTABLE_MOTOR
        )

        self.l_wrist_offset = self.l_wrist_angle - self.getMotorPosition(
            MotorConfigs.ARM_LEFT_WRIST_MOTOR
        )

        self.r_wrist_offset = self.r_wrist_offset - self.getMotorPosition(
            MotorConfigs.ARM_RIGHT_WRIST_MOTOR
        )

    def getArmAngles(self) -> None:
        self.shoulder_angle = (
            self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR) + self.shoulder_offset
        )

        self.elbow_angle = self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR) + self.elbow_offset

        self.turntable_angle = (
            self.getMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR) + self.turntable_offset
        )

        self.l_wrist_angle = (
            self.getMotorPosition(MotorConfigs.ARM_LEFT_WRIST_MOTOR) + self.l_wrist_offset
        )

        self.r_wrist_offset = (
            self.getMotorPosition(MotorConfigs.ARM_RIGHT_WRIST_MOTOR) + self.r_wrist_offset
        )

    def getMotorPosition(self, motor: MoteusMotorConfig) -> float:
        """
        Returns the current position of the given motor

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor whose position we want
        """
        position = self._info.getMotorState(motor).position
        position_radians = (position if position is not None else 0.0) * self.REVS_TO_RADIANS
        return position_radians
