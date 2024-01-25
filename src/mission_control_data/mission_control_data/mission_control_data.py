import sys
import rclpy
import json
import time
from rclpy.node import Node
from std_msgs.msg import String
from typing import List, Set
from robot_info.robot_info import RobotInfo

sys.path.append("/home/trickfire/urc-2023/src")

class MissionControlData(Node):

    def __init__(self):
        super().__init__('mission_control_data_node')
        self.get_logger().info("Creating mission control data node")

        # robot_info object to get data about motors
        self.robot_info = RobotInfo(self, self.robot_info_callback)

        # robot_JSON to store data
        self.robot_json= RobotJSON([])
        
        # publisher 
        self.time_stamp = None
        self.publisher = self.create_publisher(String, 'ROBOT_INFO_JSON', 10)
        self.timer = self.create_timer(0.5, self.publisher_timer_callback)

    def robot_info_callback(self, topic, data):
        # callback to recieve data from robot info
        self.get_logger().info(f"received topic: {topic}; data: {data}")
        self.robot_json.update(topic, data)
        self.time_stamp = time.time()


    def publisher_timer_callback(self):
        if self.time_stamp is not None:
          json_data = self.robot_json.serialize()
          msg = String()
          msg.data = json_data;
          self.publisher.publish(msg)
          self.time_stamp = None
          self.get_logger().info(f"publishing: {json_data}")
    
class RobotMotor(object):
    # represents a robot motor by storing the velocity, torque and temperature
    def __init__(self, name:str):
        self.name = name
        self.velocity = None
        self.torque = None
        self.temperature = None

    def update(self, topic:str, data:str):
        if "velocity" in topic:
            self.velocity = data
        elif "torque" in topic:
            self.torque = data
        elif "temperature" in topic:
            self.temperature = data
        
class RobotJSON(object):
    # class containing data about motors to be serialized or deserialized as JSON
    def __init__(self, motors:[]):
        self.motors = [
            RobotMotor("front_left"),
            RobotMotor("front_right"),
            RobotMotor("mid_left"),
            RobotMotor("mid_right"),
            RobotMotor("back_left"),
            RobotMotor("back_right")
        ]

    def update(self, topic:str, data:str):
        # update JSON data with motor information
        index = self.getMotorIndex(topic)
        self.motors[index].update(topic, data)
        # motor = self.motors[index]
        # motor.update(motor, topic, data)

    def serialize(self):
        # serialize data
        json_data = json.dumps(self, default=lambda o: o.__dict__, indent=4)
        return json_data
    
    def deSerialize(self, json_data: str):
        # deserialize data
        json_obj = RobotJSON(**json.loads(json_data))
        self.get_logger().info(json_obj)
        return json_obj
    
    def getMotorIndex(topic: str):
        # assigns motor index based on topic name
        if "front_left" in topic:
            return 0
        elif "front_right" in topic:
            return 1
        elif "mid_left" in topic:
            return 2
        elif "mid_right" in topic:
            return 3
        elif "back_left" in topic:
            return 4
        elif "back_right" in topic:
            return 5
        else:
            return -1

def main(args=None):
    rclpy.init(args=args)
    mission_control_data_node = MissionControlData()
    rclpy.spin(mission_control_data_node)
    mission_control_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
