"""
This module just contains the RobotInterface class. It provides utility functions to interact with
motors on the robot. 
"""

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusRunSettings


class RobotInterface:
    """
    A class that provides utility functions to interact with motors on the robot
    """

    def __init__(self, ros_node: Node) -> None:
        self._ros_node = ros_node
        self._publishers: dict[int, Publisher] = {}

        for motor_config in MotorConfigs.getAllMotors():
            self._ros_node.create_publisher(String, motor_config.getInterfaceTopicName(), 10)

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
        self.runMotor(motor, MoteusRunSettings(velocity=target_radians_per_second))

    def runMotorPosition(self, motor: MoteusMotorConfig, target_revolutions: float) -> None:
        """
        Runs the specified motor to reach the specified position.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to run.
        target_revolutions: float
            The target position in revolutions.
        """
        self.runMotor(motor, MoteusRunSettings(position=target_revolutions))

    def stopMotor(self, motor: MoteusMotorConfig) -> None:
        """
        Stops the specified motor.

        Parameters
        -------
        motor : MoteusMotorConfig
            The config of the motor to stop.
        """
        self.runMotor(motor, MoteusRunSettings(velocity=0, set_stop=True))
