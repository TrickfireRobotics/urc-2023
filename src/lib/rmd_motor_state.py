"""
This module contains objects representing the RMD-X8 motor state and settings for `set_position`.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class RMDX8MotorState:
    """
    A dataclass representing the state of the RMD-X8 motor.
    """

    can_id: int = -1
    position: Optional[float] = None
    """
    Current position of the motor in degrees.
    """
    speed: Optional[float] = None
    """
    Current speed of the motor in degrees per second.
    """
    torque: Optional[float] = None
    """
    Current torque of the motor in newton meters.
    """
    temperature: Optional[float] = None
    """
    Current temperature of the controller in Celsius.
    """
    input_voltage: Optional[float] = None
    """
    Current voltage of the controller in volts.
    """
    error_code: Optional[int] = None
    """
    Current error code reported by the motor.
    """

    @classmethod
    def fromRMDX8Data(cls, can_id: int, rmdx8_data: dict) -> "RMDX8MotorState":
        """
        Creates an RMDX8MotorState from the RMD-X8 data response.
        """
        return cls(
            can_id=can_id,
            position=rmdx8_data.get("position"),
            speed=rmdx8_data.get("speed"),
            torque=rmdx8_data.get("torque"),
            temperature=rmdx8_data.get("temperature"),
            input_voltage=rmdx8_data.get("input_voltage"),
            error_code=rmdx8_data.get("error_code"),
        )


@dataclass(frozen=True)
class RMDX8RunSettings:
    """
    A dataclass representing the different settings while running an RMD-X8 motor.
    """

    position: Optional[float] = None
    """
    Requested position of the motor in degrees.
    """
    speed: Optional[float] = None
    """
    Requested speed of the motor in degrees per second.
    """
    torque_limit: Optional[float] = None
    """
    Maximum allowable torque in newton meters.
    """
    kp: Optional[float] = None
    """
    Proportional gain for position control.
    """
    kd: Optional[float] = None
    """
    Derivative gain for position control.
    """
    enable_motor: bool = True
    """
    Whether the motor should be actively controlled or stopped.
    """

    def toCommand(self) -> bytes:
        """
        Converts the settings to a byte command for the RMD-X8 protocol.
        """
        # Example command structure; actual structure depends on RMD-X8 protocol
        command = [0xA1]  # Example base command
        if self.position is not None:
            command.extend(self.floatToBytes(self.position, scale=100))
        if self.speed is not None:
            command.extend(self.floatToBytes(self.speed, scale=10))
        if self.torque_limit is not None:
            command.extend(self.floatToBytes(self.torque_limit, scale=1))
        command.append(0x01 if self.enable_motor else 0x00)  # Motor enable/disable
        return bytes(command)

    @staticmethod
    def floatToBytes(value: float, scale: float = 1.0) -> list[int]:
        """
        Converts a float to bytes with scaling for the motor command.
        """
        scaled_value = int(value * scale)
        return [(scaled_value >> 8) & 0xFF, scaled_value & 0xFF]
