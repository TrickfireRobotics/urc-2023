"""
This module just contains the RobotInterface class. It provides utility functions to interact with
motors on the robot. 
"""

import json
import math

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusRunSettings

REVS_TO_RADIANS = math.pi * 2
RADIANS_TO_REVS = 1 / REVS_TO_RADIANS


class RobotInterface:
    """
    A class that provides utility functions to interact with motors on the robot
    """

    def __init__(self, ros_node: Node) -> None:
        self._ros_node = ros_node
        self._publishers: dict[int, Publisher] = {}
        self._temp_light_publisher = self._ros_node.create_publisher(
            String, "temp_light_from_interface", 10
        )

        for motor_config in MotorConfigs.getAllMotors():
            self._publishers[motor_config.can_id] = self._ros_node.create_publisher(
                String, motor_config.getInterfaceTopicName(), 10
            )

    def runMotor(self, motor: MoteusMotorConfig, run_settings: MoteusRunSettings) -> None:
        """
        Runs the specified motor with the specified settings.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to run.
        run_settings : MoteusRunSettings
            The settings to run the motor with.
        """
        self._publishers[motor.can_id].publish(run_settings.toMsg())

    def runMotorSpeed(self, motor: MoteusMotorConfig, target_radians_per_second: float) -> None:
        """
        Runs the specified motor with the specified target speed.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_radians_per_second: float
            The target speed in radians per second to run the motor.
        """
        self.runMotor(
            motor,
            MoteusRunSettings(
                position=math.nan,
                velocity=target_radians_per_second * RADIANS_TO_REVS,
                set_stop=False,
            ),
        )

    def runMotorPosition(self, motor: MoteusMotorConfig, target_radians: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_radians: float
            The target position in revolutions.
        """
        self.runMotor(
            motor, MoteusRunSettings(position=target_radians * RADIANS_TO_REVS, set_stop=False)
        )

    def stopMotor(self, motor: MoteusMotorConfig) -> None:
        """
        Stops the specified motor. This will stop the motor in place, not make it go limp unlike
        `disableMotor()`.

        Parameters
        ------
        motor: MoteusMotorConfig
            The config of the motor to stop.
        """
        self.runMotor(motor, MoteusRunSettings(position=math.nan, velocity=0, set_stop=False))

    def disableMotor(self, motor: MoteusMotorConfig) -> None:
        """
        Disables the specified motor. This essentially turns off the motor, stopping all output and
        making it go limp.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to disable.
        """
        self.runMotor(motor, MoteusRunSettings(velocity=0, set_stop=True))

    def setTempLight(self, target: String, state: int) -> None:
        """
        Used for the SAR autonav code to demonstrate the different states of the rover's autonomous code.

        Parameters
        -------
        target : String
            What light to set the LED string.
            Valid options are:
            'RED'
            'BLUE'
            'GREEN'

        state : int
            Sets the state of the LED strip. Will override previous target states
            0 -> OFF
            1 -> ON
        """

        temp = {"target": target, "state": state}

        msg = String()
        msg.data = "" + json.dumps(temp)
        self._temp_light_publisher.publish(msg)
