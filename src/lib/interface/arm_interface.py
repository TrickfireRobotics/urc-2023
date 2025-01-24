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
        self.torque_elbow = self._info.getMotorState(MotorConfigs.ARM_ELBOW_MOTOR).torque
        self.torque_shoulder = self._info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR).torque
        self.feedforward = 0.0

        for motor_config in MotorConfigs.getAllMotors():
            self._publishers[motor_config.can_id] = self._ros_node.create_publisher(
                String, motor_config.getInterfaceTopicName(), 10
            )

    def runArmElbowMotorPosition(self, motor: MoteusMotorConfig, target_radians: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------`
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_radians: float
            The target position in revolutions.
        feed_forward: float
            The FeedForward adjustment for the motors.
        """
        if self.target_elbow is not None and self.target_shoulder is not None:
            if self.torque_elbow is not None and self.torque_shoulder is not None:
                if self.target_shoulder < 0.0 and self.target_shoulder > -0.25:
                    self.feedforward = (
                        (56.14025 * (math.cos(self.target_elbow)))
                        - (30.13 * (math.cos(self.target_shoulder)))
                        - (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.25 and self.target_shoulder > -0.50:
                    self.feedforward = (
                        (56.14025 * (math.cos(self.target_elbow)))
                        - (30.13 * (math.cos(self.target_shoulder)))
                        - (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.50 and self.target_shoulder > -0.75:
                    self.feedforward = (
                        (56.14025 * (math.sin(self.target_elbow)))
                        - (30.13 * (math.cos(self.target_shoulder)))
                        + (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.75 and self.target_shoulder > 0.0:
                    self.feedforward = (
                        (56.14025 * (math.sin(self.target_elbow)))
                        - (30.13 * (math.cos(self.target_shoulder)))
                        - (self.torque_elbow - self.torque_shoulder)
                    )

                self._interface.runMotor(
                    motor,
                    MoteusRunSettings(
                        position=target_radians * RADIANS_TO_REVS,
                        feedforward_torque=self.feedforward,
                        set_stop=False,
                    ),
                )

    def runArmShoulderMotorPosition(self, motor: MoteusMotorConfig, target_radians: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------`
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_radians: float
            The target position in revolutions.
        feed_forward: float
            The FeedForward adjustment for the motors.
        """
        if self.target_elbow is not None and self.target_shoulder is not None:
            if self.torque_elbow is not None and self.torque_shoulder is not None:
                if self.target_shoulder < 0.0 and self.target_shoulder > -0.25:
                    self.feedforward = (
                        (30.13 * (math.cos(self.target_shoulder)))
                        - (
                            9.5
                            * (
                                5.9095 * (math.cos(self.target_elbow))
                                + 15.065 * (math.cos(self.target_shoulder))
                            )
                        )
                        - (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.25 and self.target_shoulder > -0.50:
                    self.feedforward = (
                        (30.13 * (math.cos(self.target_shoulder)))
                        - (
                            9.5
                            * (
                                7.5325 * (math.cos(self.target_shoulder))
                                - 5.9095 * (math.cos(self.target_elbow))
                            )
                        )
                        + (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.50 and self.target_shoulder > -0.75:
                    self.feedforward = (
                        (30.13 * (math.cos(self.target_shoulder)))
                        + (
                            9.5
                            * (
                                15.065 * (math.cos(self.target_shoulder))
                                - (5.9095 * (math.sin(self.target_elbow)))
                            )
                        )
                        + (self.torque_elbow - self.torque_shoulder)
                    )

                if self.target_shoulder < -0.75 and self.target_shoulder > 0.0:
                    self.feedforward = (
                        (30.13 * (math.cos(self.target_shoulder)))
                        + (
                            9.5 * (5.9095 * (math.sin(self.target_elbow)))
                            + (15.065 * (math.cos(self.target_shoulder)))
                        )
                        - (self.torque_elbow - self.torque_shoulder)
                    )

                self._interface.runMotor(
                    motor,
                    MoteusRunSettings(
                        position=target_radians * RADIANS_TO_REVS,
                        feedforward_torque=self.feedforward,
                        set_stop=False,
                    ),
                )
