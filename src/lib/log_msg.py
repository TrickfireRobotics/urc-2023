"""
This module is used for the logging of messages and dictionries transmitted and recieved by 
publishers and subscribers
"""

import datetime
from typing import Any, Generic, TypeVar

from std_msgs.msg import String

T = TypeVar("T")


class LogMsg(Generic[T]):
    """
    A collection of methods for record and retrieving records of the movement and data
    """

    def _logDictionary(
        self, node_name: String, dictionary: dict[str, Any], file_path: String
    ) -> None:
        """
        Logs a dictionary of values to a text file with a timestamp

        dictionary:  Dictionary of data to log
        file_path: Path to the file where logs should be saved

        Returns nothing
        """
        try:
            # Get the current timestamp for record
            timestamp = datetime.datetime.now().strftime("%y-%m-%d %H:%M:%S")

            # Open the file in append mode
            with open(file_path, "a", encoding="utf-8") as file:
                # Write the timestamp
                file.write(f"Timestamp: {timestamp}\n")

                # Write the Node name
                file.write(f"Node: {node_name}\n")

                # Write the dictionary key and value pairs
                for key, value in dictionary.items():
                    file.write(f"{key}: {value}\n")

                # Seperate each log with a line
                file.write("\n" + "-" * 40 + "\n\n")

            print("Logged data successfully.")
        except Exception as e:
            print(f"Error occurred with logging: {e}")
