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


class MotorConfigs:
    """
    A constants class that contains motor constants.
    """

    # Drivebase
    REAR_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=20, config={})
    MID_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=21, config={})
    FRONT_RIGHT_DRIVE_MOTOR = MoteusMotorConfig(can_id=22, config={})

    REAR_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=23, config={})
    MID_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=24, config={})
    FRONT_LEFT_DRIVE_MOTOR = MoteusMotorConfig(can_id=25, config={})

    # Arm
    ARM_SHOULDER_MOTOR = MoteusMotorConfig(can_id=1, config={})
    ARM_ELBOW_MOTOR = MoteusMotorConfig(can_id=2, config={})
    ARM_LEFT_WRIST_MOTOR = MoteusMotorConfig(can_id=3, config={})
    ARM_RIGHT_WRIST_MOTOR = MoteusMotorConfig(can_id=4, config={})
    ARM_TURNTABLE_MOTOR = MoteusMotorConfig(can_id=5, config={})

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
