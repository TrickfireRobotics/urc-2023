"""
This module contains all the relevant configs for the rover, and any classes needed to hold the
configs.
"""

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MoteusMotorConfig:
    """
    A dataclass that contains config values relating to moteus motors connected via the can bus.
    """

    can_id: int
    """
    The can id of the motor.
    """

    config: dict[str, float | int]
    """
    The config of the motor. See:
    https://github.com/mjbots/moteus/blob/main/docs/reference.md#c-configurable-values

    The `id.id` will be ignored.
    """

    def getCanTopicName(self) -> str:
        """
        Gets the motor's topic name for data sourced from can.

        Returns the following format: `moteusmotor_<can_id>_from_can`
        """
        return f"moteusmotor_{self.can_id}_from_can"

    def getInterfaceTopicName(self) -> str:
        """
        Gets the motor's topic name for data sourced from robot interface.

        Returns the following format: `moteusmotor_<can_id>_from_interface`
        """
        return f"moteusmotor_{self.can_id}_from_interface"


@dataclass(frozen=True)
class RMDx8MotorConfig:
    """
    TODO: Add comment
    """

    can_id: int
    """
    The can id of the RMDx8.
    """

    config: dict[str, float | int]
    """
    The config of the motor.
    """

    def getCanTopicName(self) -> str:
        """
        Gets the motor's topic name for data sourced from can.
        Returns the following format: 'rmdx8motor_<can_id>_from_can'
        """
        return f"rmdx8motor_{self.can_id}_from_can"

    def getInterfaceTopicName(self) -> str:
        """
        Gets the motor's topic name for data sourced from robot interface.
        Returns the following format: 'rmdx8motor_<can_id>_from_interface'
        """
        return f"rmdx8motor_{self.can_id}_from_interface"


class MotorConfigs:
    """
    A constants class that contains motor constants.
    """

    # Drivebase
    REAR_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=20,
        config={
            "servo.pwm_rate_hz": 50000,
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    MID_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=21,
        config={
            "servo.pwm_rate_hz": 50000,
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    FRONT_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=22,
        config={
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )

    REAR_LEFT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=23,
        config={
            "servo.pwm_rate_hz": 50000,
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    MID_LEFT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=24,
        config={
            "servo.pwm_rate_hz": 50000,
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    FRONT_LEFT_DRIVE_MOTOR = MoteusMotorConfig(
        can_id=25,
        config={
            "servo.pwm_rate_hz": 50000,
            "servo.pid_position.kp": 20.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 0.5,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 100.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )

    # Arm
    ARM_SHOULDER_MOTOR = MoteusMotorConfig(
        can_id=1,
        config={
            "servo.max_voltage": 56.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    ARM_ELBOW_MOTOR = MoteusMotorConfig(
        can_id=2,
        config={
            "motor_position.rotor_to_output_ratio": 0.01666,
            "servo.pwm_rate_hz": 60000,
            "servo.pid_position.kp": 10000.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 0.0,
            "servo.default_timeout_s": 1.0,
            "servo.max_current_A": 6.0,
            "servo.max_velocity": 0.1,
            "servopos.position_min": -0.5,
            "servopos.position_max": 0.0,
        },
    )
    ARM_LEFT_WRIST_MOTOR = MoteusMotorConfig(can_id=3, config={})
    ARM_RIGHT_WRIST_MOTOR = MoteusMotorConfig(can_id=4, config={})
    ARM_TURNTABLE_MOTOR = MoteusMotorConfig(
        can_id=5,
        config={
            "motor_position.rotor_to_output_ratio": 0.030581,
            "servo.pid_position.kp": 16.0,
            "servo.pid_position.ki": 0.0,
            "servo.pid_position.kd": 1.0,
            "servo.max_current_A": 10.0,
            "servo.max_velocity": 1.0,
            "servopos.position_min": math.nan,
            "servopos.position_max": math.nan,
        },
    )
    # Temporary config variable for testing for RMDx8
    RMDx8_TESTING_MOTOR = RMDx8MotorConfig(can_id=6, config={})

    # Don't allow anyone to change this class's attributes
    def __setattr__(self, _: str, __: Any) -> Any:
        raise AttributeError("Trying to set attribute on a frozen instance")

    @classmethod
    def getAllMotors(cls) -> list[MoteusMotorConfig]:
        """
        Returns a list of every motor in this constants class.
        """
        # Simply return every attribute that doesn't start with an underscore and isn't this method
        return [
            value
            for key, value in cls.__dict__.items()
            if not key.startswith("_") and key != "getAllMotors"
        ]
