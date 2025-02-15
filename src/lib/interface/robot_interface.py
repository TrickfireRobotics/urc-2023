"""
This module just contains the RobotInterface class. It provides utility functions to interact with
motors on the robot. 
"""

import math

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs, LightConfig, ServoConfig, StepperConfig
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

        for motor_config in MotorConfigs.getAllMotors():
            self._publishers[motor_config.can_id] = self._ros_node.create_publisher(
                String, motor_config.getInterfaceTopicName(), 10
            )

        # Initialize publishers for lights
        for light_config in MotorConfigs.getAllLights():
            self._publishers[light_config.can_id] = self._ros_node.create_publisher(
                String, light_config.getCanTopicName(), 10
            )

        # Initialize publishers for servos
        for servo_config in MotorConfigs.getAllServos():
            self._publishers[servo_config.can_id] = self._ros_node.create_publisher(
                String, servo_config.getCanTopicName(), 10
            )

        # Initialize publishers for steppers
        for stepper_config in MotorConfigs.getAllSteppers():
            self._publishers[stepper_config.can_id] = self._ros_node.create_publisher(
                String, stepper_config.getCanTopicName(), 10
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

    # ------ LIGHTS ------
    def setLightState(self, light: LightConfig, state: str) -> bool:
        """
        Sets the state of the specified light.

        Parameters
        -------
        light : LightConfig
            The config of the light to set the state for.
        state : str
            The state to set the light to. Should be either "TURN_ON" or "TURN_OFF".

        Returns
        -------
        bool
            True if the command was sent successfully, False otherwise.
        """
        if light.can_id in self._publishers and state in ["TURN_ON", "TURN_OFF"]:
            str_msg = String()
            str_msg.data = state
            self._publishers[light.can_id].publish(str_msg)
            return True
        return False

    # ------ Steppers ------
    def setStepperMotorState(self, stepper: StepperConfig, command: str, position: float = math.nan, speed: float = math.nan) -> bool:
        """
        Sets the state of the specified stepper motor.

        Parameters
        -------
        stepper : StepperConfig
            The config of the stepper motor to set the state for.
        command : str
            The command to set the stepper motor to. Should be one of "RUN", "SPEED", "POSITION", "STOP", "DISABLE".
        position : float, optional
            The target position for the stepper motor, if applicable.
        speed : float, optional
            The target speed for the stepper motor, if applicable.

        Returns
        -------
        bool
            True if the command was sent successfully, False otherwise.
        """
        if stepper.can_id in self._publishers and command in ["RUN", "SPEED", "POSITION", "STOP", "DISABLE"]:
            str_msg = String()
            if command == "SPEED" and not math.isnan(speed):
                str_msg.data = f"{command}:{speed}"
            elif command == "POSITION" and not math.isnan(position):
                str_msg.data = f"{command}:{position}"
            else:
                str_msg.data = command
            self._publishers[stepper.can_id].publish(str_msg)
            return True
        return False

    # ------ Servos ------
    def setServoMotorState(self, servo: ServoConfig, command: str, position: float = math.nan, speed: float = math.nan) -> bool:
        """
        Sets the state of the specified servo motor.

        Parameters
        -------
        servo : ServoConfig
            The config of the servo motor to set the state for.
        command : str
            The command to set the servo motor to. Should be one of "RUN", "SPEED", "POSITION", "STOP", "DISABLE".
        position : float, optional
            The target position for the servo motor, if applicable.
        speed : float, optional
            The target speed for the servo motor, if applicable.

        Returns
        -------
        bool
            True if the command was sent successfully, False otherwise.
        """
        if servo.can_id in self._publishers and command in ["RUN", "SPEED", "POSITION", "STOP", "DISABLE"]:
            str_msg = String()
            if command == "SPEED" and not math.isnan(speed):
                str_msg.data = f"{command}:{speed}"
            elif command == "POSITION" and not math.isnan(position):
                str_msg.data = f"{command}:{position}"
            else:
                str_msg.data = command
            self._publishers[servo.can_id].publish(str_msg)
            return True
        return False
    