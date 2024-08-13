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
        
        
        
        
        
    def buildJSONString(self):
        pythonDict = {
            "canID": self.canID,
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "temperature": self.temperature,
            "power": self.power,
            "inputVoltage": self.inputVoltage,
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
        self.inputVoltage = pythonDict["inputVoltage"]
        

        
        