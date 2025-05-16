import time

from rclpy.node import Node
from rclpy.publisher import Publisher
from smbus2 import SMBus  # Smbus has cool i2c functions
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.moteus_motor_state import MoteusRunSettings

from .robot_info import RobotInfo
from .robot_interface import RobotInterface


class i2C_Cooling:

    def __init__(self, ros_node: Node, info: RobotInfo, interface: RobotInterface) -> None:
        self._ros_node = ros_node
        self._publishers: dict[int, Publisher] = {}
        self._info = info

        self.tempShoulder = self._info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR).temperature
        self.tempElbow = self._info.getMotorState(MotorConfigs.ARM_ELBOW_MOTOR).temperature
        self.tempWrist1 = self._info.getMotorState(MotorConfigs.ARM_WRIST1_MOTOR).temperature
        self.tempWrist2 = self._info.getMotorState(MotorConfigs.ARM_WRIST2_MOTOR).temperature

        for motor_config in MotorConfigs.getAllMotors():
            self._publishers[motor_config.can_id] = self._ros_node.create_publisher(
                String, motor_config.getInterfaceTopicName(), 10
            )

    def turnOnFans(self) -> None:
        i2c_bus = SMBus(1)  # whatever i2c bus we pick
        device_address = 0x10

        while True:

            if self.tempShoulder > 50:
                fanOffandOn |= 0b0001
            else:
                fanOffandOn &= 0b1110
            if self.tempElbow > 50:
                fanOffandOn |= 0b0010
            else:
                fanOffandOn &= 0b1101
            if self.tempWrist1 > 50:
                fanOffandOn |= 0b0100
            else:
                fanOffandOn &= 0b1011
            if self.tempWrist2 > 50:
                fanOffandOn |= 0b1000
            else:
                fanOffandOn &= 0b0111

            i2c_bus.write_byte_data(device_address, 0, fanOffandOn)  # Writing to the i2c bus

            time.sleep(1)
