"""
This module contains the class which allows for controlling the arm motors individually with
position. It is accessed by the arm node (arm.py) to allow for controls from this class when
we switch to mode Position Control from mission control.
"""

import math

from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


class IndividualControlPos:
    """
    Class that uses subscriber callback methods to move the arm by incrementing and decrementing
    the target positions of the individual motors.
    """

    REVS_TO_RADIANS = math.pi * 2.0

    def __init__(self, ros_node: Node, interface: RobotInterface, info: RobotInfo):
        self.can_send = False
        self._ros_node = ros_node
        self._bot_interface = interface
        self._bot_info = info

        self.target_shoulder_pos = self.getMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR)
        self.target_elbow_pos = self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR)

        # elbow subscribers
        self.elbow_up_sub = ros_node.create_subscription(Float32, "elbow_up", self.elbowUp, 10)

        self.elbow_down_sub = ros_node.create_subscription(
            Float32, "elbow_down", self.elbowDown, 10
        )

        # shoulder subscribers
        self.shoulder_up_sub = ros_node.create_subscription(
            Float32, "shoulder_up", self.shoulderUp, 10
        )

        self.shoulder_down_sub = ros_node.create_subscription(
            Float32, "shoulder_down", self.shoulderDown, 10
        )

    def elbowUp(self, msg: Float32) -> None:
        """
        Called when messages from elbow up are published
        Increases the elbow target position and runs motor to that position
        """
        if not self.can_send or msg == 0.0:
            return
        self.target_elbow_pos -= 0.1
        self._ros_node.get_logger().info("elbow up: " + str(self.target_elbow_pos))
        self._bot_interface.runMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR, self.target_elbow_pos)

    def elbowDown(self, msg: Float32) -> None:
        """
        Called when messages from elbow down are published
        Decreases the elbow target position and runs motor to that position
        """
        if not self.can_send or msg == 0.0:
            return
        self.target_elbow_pos += 0.1
        self._ros_node.get_logger().info("elbow down: " + str(self.target_elbow_pos))
        self._bot_interface.runMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR, self.target_elbow_pos)

    def shoulderUp(self, msg: Float32) -> None:
        """
        Called when messages from shoulder up are published
        Increases shoulder target position and runs motor to that position
        """
        if not self.can_send or msg == 0.0:
            return
        self.target_shoulder_pos -= 0.02
        self._ros_node.get_logger().info("shoulder up: " + str(self.target_shoulder_pos))
        self._bot_interface.runMotorPosition(
            MotorConfigs.ARM_SHOULDER_MOTOR, self.target_shoulder_pos
        )

    def shoulderDown(self, msg: Float32) -> None:
        """
        Called when messages from shoulder down are published
        Decreases shoulder target position and runs motor to that position
        """
        if not self.can_send or msg == 0.0:
            return
        self.target_shoulder_pos += 0.02
        self._ros_node.get_logger().info("shoulder down: " + str(self.target_shoulder_pos))
        self._bot_interface.runMotorPosition(
            MotorConfigs.ARM_SHOULDER_MOTOR, self.target_shoulder_pos
        )

    def getMotorPosition(self, motor: MoteusMotorConfig) -> float:
        """
        Returns the current position of the given motor

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor whose position we want
        """
        position = self._bot_info.getMotorState(motor).position
        position_radians = (position if position is not None else 0.0) * self.REVS_TO_RADIANS
        return position_radians
