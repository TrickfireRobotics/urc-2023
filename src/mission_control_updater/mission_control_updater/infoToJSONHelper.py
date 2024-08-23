import json
import array


class InfoToJSONHelper():
    
    def __init__(self):
        self.moteusEntries = []
        
    
    
    def addMoteusEntry(self, entry):
        self.moteusEntries.append(entry)
    
    def buildJSONString(self):
        moteusDictArray = []
        
        for moteusEntry in self.moteusEntries:
            moteusDictArray.append(moteusEntry.buildPythonDict())
        
        pythonDict = {
            "moteusMotorLength": len(moteusDictArray),
            "moteusMotors": moteusDictArray
        }
        
        return json.dumps(pythonDict)