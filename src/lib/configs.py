"""
This module contains all the relevant configs for the rover, and any classes needed to hold the
configs.
"""

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


class MotorConfigs:
    """
    A constants class that contains motor constants.
    """

    # Drivebase
    REAR_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=20)
    MID_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=21)
    FRONT_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=22)

    REAR_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=23)
    MID_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=24)
    FRONT_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=25)

    # Arm
    ARM_SHOULDER_MOTOR = MoteusMotorConfig(can_id=1)
    ARM_ELBOW_MOTOR = MoteusMotorConfig(can_id=2)
    ARM_LEFT_WRIST_MOTOR = MoteusMotorConfig(can_id=3)
    ARM_RIGHT_WRIST_MOTOR = MoteusMotorConfig(can_id=4)
    ARM_TURNTABLE_MOTOR = MoteusMotorConfig(can_id=5)

    # Don't allow anyone to change this class's attributes
    def __setattr__(self, _: str, __: Any) -> Any:
        raise AttributeError("Trying to set attribute on a frozen instance")

    def getAllMotors(self) -> list[MoteusMotorConfig]:
        """
        Returns a list of every motor in this constants class.
        """
        # Simply return every attribute that doesn't start with an underscore
        return [value for key, value in self.__dict__.items() if not key.startswith("_")]
