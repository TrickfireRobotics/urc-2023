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
class LightConfig:
    """
    A dataclass that contains config values relating to lights connected via the can bus.
    """

    can_id: int
    """
    The can id of the light.
    """

    config: dict[str, float | int]
    """
    The config of the light.
    """

    def getCanTopicName(self) -> str:
        """
        Gets the light's topic name for data sourced from can.

        Returns the following format: `light_<can_id>_from_can`
        """
        return f"light_{self.can_id}_from_can"


@dataclass(frozen=True)
class ServoConfig:
    """
    A dataclass that contains config values relating to servos connected via the can bus.
    """

    can_id: int
    """
    The can id of the servo.
    """

    config: dict[str, float | int]
    """
    The config of the servo.
    """

    def getCanTopicName(self) -> str:
        """
        Gets the servo's topic name for data sourced from can.

        Returns the following format: `servo_<can_id>_from_can`
        """
        return f"servo_{self.can_id}_from_can"


@dataclass(frozen=True)
class StepperConfig:
    """
    A dataclass that contains config values relating to stepper motors connected via the can bus.
    """

    can_id: int
    """
    The can id of the stepper motor.
    """

    config: dict[str, float | int]

    ms1: int 
    ms2: int 
    ms3: int 

    """
    The config of the stepper motor.
    """

    def getCanTopicName(self) -> str:
        """
        Gets the stepper motor's topic name for data sourced from can.

        Returns the following format: `stepper_<can_id>_from_can`
        """
        return f"stepper_{self.can_id}_from_can"


@dataclass(frozen=True)
class MotherboardConfig:
    """
    A dataclass that contains variable names and configuration IDs for motherboard components.
    """
    # Light configurations
    LIGHT_RED = "red"
    LIGHT_GREEN = "green"
    LIGHT_BLUE = "blue"
    LIGHT_BRIGHTNESS = "brightness"
    LIGHT_FREQUENCY_HZ = "frequency_hz"

    # Servo configurations
    SERVO_0 = "servo_0"
    SERVO_1 = "servo_1"
    SERVO_2 = "servo_2"
    SERVO_3 = "servo_3"
    SERVO_4 = "servo_4"
    SERVO_5 = "servo_5"
    SERVO_PWM_RATE_HZ = "pwm_rate_hz"
    SERVO_MAX_ANGLE = "max_angle"
    SERVO_MIN_ANGLE = "min_angle"

    # Stepper configurations
    STEPPER_0 = "stepper_0"
    STEPPER_1 = "stepper_1"
    STEPPER_2 = "stepper_2"
    STEPPER_3 = "stepper_3"
    STEPPER_4 = "stepper_4"
    STEPPER_5 = "stepper_5"
    STEPPER_MAX_STEPS = "max_steps"
    STEPPER_STEP_RATE = "step_rate"


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

    # Lights
    LIGHT_RED = LightConfig(
        can_id=30,
        config={
            MotherboardConfig.LIGHT_BRIGHTNESS: 255,
            MotherboardConfig.LIGHT_FREQUENCY_HZ: 100,
        },
    )
    LIGHT_GREEN = LightConfig(
        can_id=31,
        config={
            MotherboardConfig.LIGHT_BRIGHTNESS: 255,
            MotherboardConfig.LIGHT_FREQUENCY_HZ: 100,
        },
    )
    LIGHT_BLUE = LightConfig(
        can_id=32,
        config={
            MotherboardConfig.LIGHT_BRIGHTNESS: 255,
            MotherboardConfig.LIGHT_FREQUENCY_HZ: 100,
        },
    )

    # Servos
    SERVO_0 = ServoConfig(
        can_id=40,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )
    SERVO_1 = ServoConfig(
        can_id=41,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )
    SERVO_2 = ServoConfig(
        can_id=42,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )
    SERVO_3 = ServoConfig(
        can_id=43,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )
    SERVO_4 = ServoConfig(
        can_id=44,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )
    SERVO_5 = ServoConfig(
        can_id=45,
        config={
            MotherboardConfig.SERVO_PWM_RATE_HZ: 50,
            MotherboardConfig.SERVO_MAX_ANGLE: 180.0,
            MotherboardConfig.SERVO_MIN_ANGLE: 0.0,
        },
    )

    # Steppers
    STEPPER_0 = StepperConfig(
        can_id=50,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )
    STEPPER_1 = StepperConfig(
        can_id=51,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )
    STEPPER_2 = StepperConfig(
        can_id=52,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )
    STEPPER_3 = StepperConfig(
        can_id=53,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )
    STEPPER_4 = StepperConfig(
        can_id=54,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )
    STEPPER_5 = StepperConfig(
        can_id=55,
        config={
            MotherboardConfig.STEPPER_MAX_STEPS: 200,
            MotherboardConfig.STEPPER_STEP_RATE: 1000,
        },

        ms1=0,
        ms2=0,
        ms3=0,
    )


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
    
    @classmethod
    def getAllLights(cls) -> list[LightConfig]:     
        """
        Returns a list of every light in this constants class.
        """
        return [
            value
            for key, value in cls.__dict__.items()
            if not key.startswith("_") and isinstance(value, LightConfig)
        ]

    @classmethod
    def getAllServos(cls) -> list[ServoConfig]:
        """
        Returns a list of every servo in this constants class.
        """
        return [
            value
            for key, value in cls.__dict__.items()
            if not key.startswith("_") and isinstance(value, ServoConfig)
        ]

    @classmethod
    def getAllSteppers(cls) -> list[StepperConfig]:
        """
        Returns a list of every stepper motor in this constants class.
        """
        return [
            value
            for key, value in cls.__dict__.items()
            if not key.startswith("_") and isinstance(value, StepperConfig)
        ]