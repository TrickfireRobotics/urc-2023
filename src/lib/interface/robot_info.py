"""
This module just contains the RobotInfo class. It provides utility functions to access the state of
motor on the robot.
"""

from typing import Any

from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String

from custom_interfaces.srv import MoteusState
from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusMotorState


class RobotInfo:  # pylint: disable=too-few-public-methods
    """
    A class that provides utility functions to access the state of motors within the robot.
    """

    def __init__(self, ros_node: Node):
        self._ros_node = ros_node
        self.sub_list: list[Subscription] = []  # empty array
        self.can_id_to_json: dict[int, MoteusMotorState] = {}  # Dict

        self.get_moteus_state_service = self._ros_node.create_service(
            MoteusState, "get_moteus_motor_state", self._handleGetMoteusState
        )

        for motor_config in MotorConfigs.getAllMotors():
            self._ros_node.create_subscription(
                String, motor_config.getCanTopicName(), self._subCallback, 10
            )
            self.can_id_to_json[motor_config.can_id] = MoteusMotorState()

    def _subCallback(self, msg: String) -> None:
        state = MoteusMotorState.fromJsonMsg(msg)
        self.can_id_to_json[state.can_id] = state

    def _handleGetMoteusState(
        self, request: MoteusState.Request, response: MoteusState.Response
    ) -> MoteusState.Response:

        can_id = request.target_can_id
        string_message = self.can_id_to_json[can_id].toMsg()
        response.json_payload = str(string_message.data)
        return response

    def getMotorState(self, motor: MoteusMotorConfig) -> MoteusMotorState:
        """
        Gets the state of the motor with the given can_id.

        Parameters
        ------
        can_id: int
            The can id of the motor to get the state of.
        """
        return self.can_id_to_json[motor.can_id]
