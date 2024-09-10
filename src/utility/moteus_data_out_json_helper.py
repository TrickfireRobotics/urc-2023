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
        self.inputVoltage = None
        self.qCurrent = None
        self.dCurrent = None
        
        
        
        
        
    def buildJSONString(self):
        pythonDict = self.buildPythonDict()
        
        return json.dumps(pythonDict)
    
    def buildPythonDict(self):
        pythonDict = {
            "canID": self.canID,
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "temperature": self.temperature,
            "power": self.power,
            "inputVoltage": self.inputVoltage,
            "qCurrent": self.qCurrent,
            "dCurrent": self.dCurrent
        }
        
        return pythonDict
    
    def buildHelper(self, jsonString):
        pythonDict = json.loads(jsonString)
        
        self.canID = pythonDict["canID"]
        self.position = pythonDict["position"]
        self.velocity = pythonDict["velocity"]
        self.torque = pythonDict["torque"]
        self.temperature = pythonDict["temperature"]
        self.power = pythonDict["power"]
        self.inputVoltage = pythonDict["inputVoltage"]
        self.qCurrent = pythonDict["qCurrent"]
        self.dCurrent = pythonDict["dCurrent"]
        

        
        