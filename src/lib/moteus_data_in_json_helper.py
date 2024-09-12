import json
import math


class MoteusDataInJsonHelper:

    def __init__(self) -> None:
        self.position: float | None = None
        self.velocity: float | None = None
        self.feedforward_torque: float | None = None
        self.kp_scale: float | None = None
        self.kd_scale: float | None = None
        self.max_torque: float | None = None
        self.watchdog_timeout: float | None = None
        self.velocity_limit: float | None = None
        self.accel_limit: float | None = None
        self.fixed_voltage_override: float | None = None

        self.set_stop = True

    def getPosition(self) -> float:
        if self.position is None:
            return math.nan

        return self.position

    def getVelocity(self) -> float:
        if self.velocity is None:
            return math.nan

        return self.velocity

    def buildJSONString(self) -> str:
        python_dict = {
            "position": self.position,
            "velocity": self.velocity,
            "feedforward_torque": self.feedforward_torque,
            "kp_scale": self.kp_scale,
            "kd_scale": self.kd_scale,
            "max_torque": self.max_torque,
            "watchdog_timeout": self.watchdog_timeout,
            "velocity_limit": self.velocity_limit,
            "accel_limit": self.accel_limit,
            "fixed_voltage_override": self.fixed_voltage_override,
            "set_stop": self.set_stop,
        }

        return json.dumps(python_dict)

    def buildHelper(self, json_string: str) -> None:
        python_dict = json.loads(json_string)

        self.position = python_dict["position"]
        self.velocity = python_dict["velocity"]
        self.feedforward_torque = python_dict["feedforward_torque"]
        self.kp_scale = python_dict["kp_scale"]
        self.kd_scale = python_dict["kd_scale"]
        self.max_torque = python_dict["max_torque"]
        self.watchdog_timeout = python_dict["watchdog_timeout"]
        self.velocity_limit = python_dict["velocity_limit"]
        self.accel_limit = python_dict["accel_limit"]
        self.fixed_voltage_override = python_dict["fixed_voltage_override"]
        self.set_stop = python_dict["set_stop"]
