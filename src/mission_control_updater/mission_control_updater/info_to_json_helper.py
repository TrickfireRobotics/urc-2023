import json

from lib.moteus_data_out_json_helper import MoteusDataOutJsonHelper


class InfoToJSONHelper:

    def __init__(self) -> None:
        self.moteus_entries: list[MoteusDataOutJsonHelper] = []

    def addMoteusEntry(self, entry: MoteusDataOutJsonHelper) -> None:
        self.moteus_entries.append(entry)

    def buildJSONString(self) -> str:
        moteus_dict_array = []

        for moteus_entry in self.moteus_entries:
            moteus_dict_array.append(moteus_entry.buildPythonDict())

        python_dict = {
            "moteusMotorLength": len(moteus_dict_array),
            "moteusMotors": moteus_dict_array,
        }

        return json.dumps(python_dict)
