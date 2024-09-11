import json


class MoteusDataOutJsonHelper:

    def __init__(self) -> None:
        # take a look at moteus.Register
        self.can_id = -1
        self.position: float | None = None
        self.velocity: float | None = None
        self.torque: float | None = None
        self.temperature: float | None = None
        self.power: float | None = None
        self.input_voltage: float | None = None
        self.q_current: float | None = None
        self.d_current: float | None = None

    def buildJSONString(self) -> str:
        python_dict = self.buildPythonDict()

        return json.dumps(python_dict)

    def buildPythonDict(self) -> dict[str, float | None]:
        python_dict = {
            "can_id": self.can_id,
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "temperature": self.temperature,
            "power": self.power,
            "input_voltage": self.input_voltage,
            "q_current": self.q_current,
            "d_current": self.d_current,
        }

        return python_dict

    def buildHelper(self, json_string: str) -> None:
        python_dict = json.loads(json_string)

        self.can_id = python_dict["can_id"]
        self.position = python_dict["position"]
        self.velocity = python_dict["velocity"]
        self.torque = python_dict["torque"]
        self.temperature = python_dict["temperature"]
        self.power = python_dict["power"]
        self.input_voltage = python_dict["input_voltage"]
        self.q_current = python_dict["q_current"]
        self.d_current = python_dict["d_current"]
