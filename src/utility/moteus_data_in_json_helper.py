import json


class MoteusDataInJsonHelper():
    
    def __init__(self):
        self.position = None
        self.velocity = None
        self.feedfoward_torque = None
        self.kp_scale = None
        self.kd_scale = None
        self.max_torque = None
        self.watchdog_timeout = None
        self.velocity_limit = None
        self.accel_limit = None
        self.fixed_votlage_override = None
        self.ilimit_scale = None
        
        self.setStop = True
        
        
    def buildJSONString(self):
        pythonDict = {
            "position": self.position,
            "velocity": self.velocity,
            "feedforward_torque": self.feedfoward_torque,
            "kp_scale": self.kp_scale,
            "kd_scale": self.kd_scale,
            "max_torque": self.max_torque,
            "watchdog_timeout": self.watchdog_timeout,
            "velocity_imit": self.velocity_limit,
            "accel_limit": self.accel_limit,
            "fixed_voltage_override": self.fixed_votlage_override,
            "ilimit_scale": self.ilimit_scale,
            "set_stop": self.setStop
        }
        
        return json.dumps(pythonDict)
    
    
    def buildHelper(self, jsonString):
        pythonDict = json.loads(jsonString)
        
        self.position = pythonDict["position"]
        self.velocity = pythonDict["velocity"]
        self.feedfoward_torque = pythonDict["feedfoward_torque"]
        self.kp_scale = pythonDict["kp_scale"]
        self.kd_scale = pythonDict["kd_scale"]
        self.max_torque = pythonDict["max_torque"]
        self.watchdog_timeout = pythonDict["watchdog_timeout"]
        self.velocity_limit = pythonDict["velocity_limit"]
        self.accel_limit = pythonDict["accel_limit"]
        self.fixed_votlage_override = pythonDict["fixed_votlage_override"]
        self.ilimit_scale = pythonDict["ilimit_scale"]
        self.setStop = pythonDict["setStop"]
        
        