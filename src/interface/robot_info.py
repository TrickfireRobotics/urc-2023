from rclpy.node import Node
from rclpy.subscription import Subscription
from std_msgs.msg import String

from utility.moteus_data_out_json_helper import MoteusDataOutJsonHelper

moteusTopicList = {
    "front_left_drive_motor_from_can",
    "mid_left_drive_motor_from_can",
    "rear_left_drive_motor_from_can",
    "front_right_drive_motor_from_can",
    "mid_right_drive_motor_from_can",
    "rear_right_drive_motor_from_can",
    "arm_turntable_motor_from_can",
    "arm_shoulder_motor_from_can",
    "arm_elbow_motor_from_can",
    "arm_left_wrist_motor_from_can",
    "arm_right_wrist_motor_from_can",
}


class RobotInfo:

    def __init__(self, ros_node: Node):
        self._ros_node = ros_node
        self.sub_list: list[Subscription] = []  # empty array
        self.can_id_to_json: dict[int, MoteusDataOutJsonHelper] = {}  # Dict

        self.can_id_to_json[20] = MoteusDataOutJsonHelper()
        self.can_id_to_json[21] = MoteusDataOutJsonHelper()
        self.can_id_to_json[22] = MoteusDataOutJsonHelper()
        self.can_id_to_json[23] = MoteusDataOutJsonHelper()
        self.can_id_to_json[24] = MoteusDataOutJsonHelper()
        self.can_id_to_json[25] = MoteusDataOutJsonHelper()
        self.can_id_to_json[1] = MoteusDataOutJsonHelper()
        self.can_id_to_json[2] = MoteusDataOutJsonHelper()
        self.can_id_to_json[3] = MoteusDataOutJsonHelper()
        self.can_id_to_json[4] = MoteusDataOutJsonHelper()
        self.can_id_to_json[5] = MoteusDataOutJsonHelper()

        self.createSubscribers()

    def createSubscribers(self) -> None:
        for topic_name in moteusTopicList:
            sub = self._ros_node.create_subscription(
                String, topic_name, self.subCallback, 1
            )
            self.sub_list.append(sub)

    def subCallback(self, msg: String) -> None:
        json_helper = MoteusDataOutJsonHelper()
        json_helper.buildHelper(msg.data)
        self.can_id_to_json[json_helper.can_id] = json_helper

    def getDataFromCanID(self, can_id: int) -> MoteusDataOutJsonHelper:
        return self.can_id_to_json[can_id]
