"""
This module contains objects representing the rmdx8 motor state and settings for `set_position`.
"""

from dataclasses import dataclass

# import moteus
import myactuator_rmd_py as rmd

from lib.json_msg import JsonMsg

# from moteus.moteus import Result


@dataclass(frozen=True)
class RMDx8MotorState(JsonMsg["RMDx8MotorState"]):
    """
    A dataclass representing the state of the rmdx8 motor.
    """

    can_id: int = -1
    position: float | None = None
    """
    Current position of the motor in revolutions.
    """
    velocity: float | None = None
    """
    Current velocity of the motor in revolutions per second. 
    """
    torque: float | None = None
    """
    Current torque of the motor in newton meters.
    """
    temperature: float | None = None
    """
    Current temperature of the controller in celcius.
    """
    power: float | None = None
    """
    Current power draw of the controller in watts.
    """
    input_voltage: float | None = None
    """
    Current voltage of the controller in volts.
    """
    q_current: float | None = None
    """
    Current q phase measured in amps.
    """
    d_current: float | None = None
    """
    Current d phase measured in amps.
    """

    # subscriber (listener)
    @classmethod
    def fromRMDx8Data(cls, can_id: int, rmdx8_result: rmd) -> "RMDx8MotorState":
        """
        Creates a RMDx8MotorState from an rmdx8 from myactuator_rmd_py library.
        Awaiting implementation of the interface from Saharsh to obtain dictionary values
        (e.g. getPosition, etc.)
        """

        dict_ = {
            "can_id": can_id,
            #     "position": moteus_result.values[moteus.Register.POSITION],
            #     "velocity": moteus_result.values[moteus.Register.VELOCITY],
            #     "torque": moteus_result.values[moteus.Register.TORQUE],
            #     "temperature": moteus_result.values[moteus.Register.TEMPERATURE],
            #     "power": moteus_result.values[moteus.Register.POWER],
            #     "input_voltage": moteus_result.values[moteus.Register.VOLTAGE],
            #     "q_current": moteus_result.values[moteus.Register.Q_CURRENT],
            #     "d_current": moteus_result.values[moteus.Register.D_CURRENT],
        }
        return cls(**dict_)


@dataclass
class RMDx8RunSettings(JsonMsg["RMDx8RunSettings"]):
    """
    A dataclass representing the different settings while running a rmdx8 motor.
    """

    position: float | None = None
    """
    Requested position of the motor in revolutions.
    """
    velocity: float | None = None
    """
    Requested velocity of the motor in revolutions per second.
    """
    feedforward_torque: float | None = None
    kp_scale: float | None = None
    kd_scale: float | None = None
    max_torque: float | None = None
    watchdog_timeout: float | None = None
    velocity_limit: float | None = None
    accel_limit: float | None = None
    fixed_voltage_override: float | None = None
    set_stop: bool = True
