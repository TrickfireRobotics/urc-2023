"""
This module just contains the RobotInfo class. It provides utility functions to access the state of
motor on the robot.
"""

from collections import defaultdict
from typing import Callable

from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.configs import MotorConfig, MotorConfigs
from lib.motor_state.can_motor_state import CANMotorState


class RobotInfo:  # pylint: disable=too-few-public-methods
    """
    A class that provides utility functions to access the state of motors within the robot.
    """

    def __init__(self, ros_node: Node):
        self._ros_node = ros_node
        self.sub_list: list[Subscription] = []  # empty array
        self.can_id_to_json: defaultdict[str, dict[int, CANMotorState]] = defaultdict(dict)

        for motor_config in MotorConfigs.getAllMotors():
            self._ros_node.create_subscription(
                String,
                motor_config.getCanTopicName(),
                self._createSubCallback(motor_config.motor_type),
                10,
            )
            self.can_id_to_json[motor_config.motor_type][motor_config.can_id] = CANMotorState()

    def _createSubCallback(self, motor_type: str) -> Callable[[String], None]:
        def _subCallback(msg: String) -> None:
            state = CANMotorState.fromJsonMsg(msg)
            self.can_id_to_json[motor_type][state.can_id] = state

        return _subCallback

    def getMotorState(self, motor: MotorConfig) -> CANMotorState:
        """
        Gets the state of the motor with the given can_id.

        Parameters
        ------
        can_id: int
            The can id of the motor to get the state of.
        """
        return self.can_id_to_json[motor.motor_type][motor.can_id]
