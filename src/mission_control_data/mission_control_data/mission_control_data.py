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

    #topic name
    TOPIC_ROBOT_INFO_JSON = "ROBOT_INFO_JSON"

    def __init__(self):
        super().__init__('mission_control_data_node')
        self.get_logger().info("Creating mission control data node")

        # robot_info object to get data about motors
        self.robot_info = RobotInfo(self, self.robot_info_callback)

        # robot_JSON to store data
        self.robot_json= RobotJSON()
        
        # publisher 
        self.time_stamp = None
        self.publisher = self.create_publisher(String, MissionControlData.TOPIC_ROBOT_INFO_JSON, 10)
        self.timer = self.create_timer(0.5, self.publisher_timer_callback)

    def robot_info_callback(self, topic, data):
        # callback to recieve data from robot info
        self.get_logger().info(f"robot info callback received topic: {topic}; data: {data}")
        self.robot_json.update(topic, data)
        self.time_stamp = time.time()

    def publisher_timer_callback(self):
        if self.time_stamp is not None:
          json_data = self.robot_json.serialize()
          msg = String()
          msg.data = json_data
          self.publisher.publish(msg)
          self.time_stamp = None
          self.get_logger().info(f"publishing: {json_data}")
    
class RobotJSON:
    
    MOTORS = ["front_left",
              "front_right",
              "mid_left",
              "mid_right",
              "back_left",
              "back_right"]
    
    MOTOR_VELOCITY = "velocity"
    MOTOR_TORQUE = "torque"
    MOTOR_TEMPERATURE = "temperature"
    

    # class containing data about motors to be serialized or deserialized as JSON
    def __init__(self, motors=[]):
        self.motors = motors
        for m in RobotJSON.MOTORS:
            motor = RobotMotor(m)
            self.motors.append(motor)

    def update(self, topic:str, data:str):
        for i in range(len(RobotJSON.MOTORS)): 
            if RobotJSON.MOTORS[i] in topic:
                self.motors[i].update(topic, data)
                break

    def serialize(self):
        # serialize data
        json_data = json.dumps(self, default=lambda o: o.__dict__, indent=4)
        return json_data
    
    def deSerialize(json_data: str):
        # deserialize data
        json_obj = RobotJSON(**json.loads(json_data))
        return json_obj
    
class RobotMotor:
    # represents a robot motor by storing the velocity, torque and temperature
    def __init__(self, name:str):
        self.name = name
        self.velocity = None
        self.torque = None
        self.temperature = None

    def update(self, topic:str, data:str):
        if RobotJSON.MOTOR_VELOCITY in topic:
            self.velocity = data
        elif RobotJSON.MOTOR_TORQUE in topic:
            self.torque = data
        elif RobotJSON.MOTOR_TEMPERATURE in topic:
            self.temperature = data


def main(args=None):
    rclpy.init(args=args)
    mission_control_data_node = MissionControlData()
    rclpy.spin(mission_control_data_node)
    mission_control_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
