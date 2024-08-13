import json


class InfoToJSONHelper():
    
    def __init__(self):
        self.moteusEntries = {}
        self.someOtherData = 10
    
    
    def addMoteusEntry(self, entry):
        self.moteusEntries.add(entry)
    
    def buildJSONString(self):
        pythonDict = {
            "moteusMotors": self.moteusEntries,
            "someOtherData": self.someOtherData
        }
        
        return json.dumps(pythonDict)