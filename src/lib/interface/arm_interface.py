"""
This module just contains the ArmInterface class. It provides utility functions to interact with
motors on the arm.
"""

import math

from rclpy.node import Node
from rclpy.publisher import Publisher
from robot_info import RobotInfo
from robot_interface import RobotInterface
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusRunSettings

REVS_TO_RADIANS = math.pi * 2
RADIANS_TO_REVS = 1 / REVS_TO_RADIANS


class ArmInterface:

    def __init__(self, ros_node: Node, info: RobotInfo, interface: RobotInterface) -> None:
        self._ros_node = ros_node
        self._publishers: dict[int, Publisher] = {}
        self._info = info
        self.target_shoulder = self._info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR).position
        self.target_elbow = self._info.getMotorState(MotorConfigs.ARM_ELBOW_MOTOR).position
        self._interface = interface
        self.feedforward = 0.0
        self.shoulder_position = 0.0
        self.elbow_position = 0.0

        for motor_config in MotorConfigs.getAllMotors():
            self._publishers[motor_config.can_id] = self._ros_node.create_publisher(
                String, motor_config.getInterfaceTopicName(), 10
            )

    def runArmElbowMotorVelocity(self, motor: MoteusMotorConfig, target_velocity: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------`
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_velocity: float
            The target velocity in revolutions per second.
        feed_forward: float
            The FeedForward adjustment for the motors.
        """
        if self.target_elbow is not None and self.target_shoulder is not None:
            self.feedforward = 0.0

            self._interface.runMotor(
                motor,
                MoteusRunSettings(
                    velocity=target_velocity,
                    feedforward_torque=self.feedforward,
                    set_stop=False,
                ),
            )

    def runArmShoulderMotorVelocity(self, motor: MoteusMotorConfig, target_velocity: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------`
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_velocity: float
            The target velocity in revolutions per second.
        feed_forward: float
            The FeedForward adjustment for the motors.
        """
        if self.target_elbow is not None and self.target_shoulder is not None:
            self.shoulder_position = self.target_shoulder * -REVS_TO_RADIANS
            self.elbow_position = self.target_elbow * -REVS_TO_RADIANS

            if (
                self.shoulder_position > math.pi
                and self.shoulder_position - self.elbow_position > math.pi
            ):
                self.feedforward = (19.53 * (math.cos(self.shoulder_position))) + 0.15 * (
                    math.cos(self.shoulder_position - self.elbow_position)
                )

            else:
                self.feedforward = (19.53 * (math.cos(self.shoulder_position))) - 0.15 * (
                    math.cos(self.shoulder_position - self.elbow_position)
                )

            self._interface.runMotor(
                motor,
                MoteusRunSettings(
                    velocity=target_velocity,
                    feedforward_torque=self.feedforward,
                    set_stop=False,
                ),
            )
