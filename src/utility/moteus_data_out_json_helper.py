import json


class MoteusDataOutJsonHelper():
    
    def __init__(self):
        # take a look at moteus.Register
        self.canID = -1
        self.position = None
        self.velocity = None
        self.torque = None
        self.temperature = None
        
        self.power = None
        self.qCurrent = None
        self.dCurrent = None
        self.inputVoltage = None
        
        self.voltagePhaseA = None
        self.voltagePhaseB = None
        self.voltagePhaseC = None
        
        
        
        
        
    def buildJSONString(self):
        pythonDict = {
            "canID": self.canID,
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "temperature": self.temperature,
            "power": self.power,
            "qCurrent": self.qCurrent,
            "dCurrent": self.dCurrent,
            "inputVoltage": self.inputVoltage,
            "voltagePhaseA": self.voltagePhaseA,
            "voltagePhaseB": self.voltagePhaseB,
            "voltagePhaseC": self.voltagePhaseC
        }
        
        return json.dumps(pythonDict)
    
    
    def buildHelper(self, jsonString):
        pythonDict = json.loads(jsonString)
        
        self.canID = pythonDict["canID"]
        self.position = pythonDict["position"]
        self.velocity = pythonDict["velocity"]
        self.torque = pythonDict["torque"]
        self.temperature = pythonDict["temperature"]
        self.power = pythonDict["power"]
        self.qCurrent = pythonDict["qCurrent"]
        self.dCurrent = pythonDict["dCurrent"]
        self.inputVoltage = pythonDict["inputVoltage"]
        self.voltagePhaseA = pythonDict["voltagePhaseA"]
        self.voltagePhaseB = pythonDict["voltagePhaseB"]
        self.voltagePhaseC = pythonDict["voltagePhaseC"]
        

        
        