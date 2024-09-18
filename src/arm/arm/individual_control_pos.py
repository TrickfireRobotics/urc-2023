import math

from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MoteusMotorConfig, MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


class IndividualControlPosition():

    REVS_TO_RADIANS = math.pi * 2.0

    # physical min and max positions of shoulder and elbow positions
    MAX_SHOULDER_POS = math.pi * 2.0
    MIN_SHOULDER_POS = 0.0
    MAX_ELBOW_POS = math.pi * 2.0
    MIN_ELBOW_POS = 0.0

    def __init__(self, ros_node: Node, interface: RobotInterface, info: RobotInfo):
        self._ros_node = ros_node
        self._interface = interface
        self._info = info
        self.can_send = False
        self.prev_can_send = False

        # set initial target positions to motors' starting positions
        self.target_turntable_pos = self.getMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR)
        self.target_shoulder_pos = self.getMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR)
        self.target_elbow_pos = self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR)

        # create subscriptions
        self.cw_turntable_pos_sub = self._ros_node.create_subscription(
            Float32, "turntable_cw", self.cwTurntablePosition, 10
        )
        self.ccw_turntable_pos_sub = self._ros_node.create_subscription(
            Float32, "turntable_ccw", self.ccwTurntablePosition, 10
        )
        self.increase_shoulder_pos_sub = self._ros_node.create_subscription(
            Float32, "shoulder_up", self.increaseShoulderPosition, 10
        )
        self.decrease_shoulder_pos_sub = self._ros_node.create_subscription(
            Float32, "shoulder_down", self.decreaseShoulderPosition, 10
        )
        self.increase_elbow_pos_sub = self._ros_node.create_subscription(
            Float32, "elbow_up", self.increaseElbowPosition, 10
        )
        self.decrease_elbow_pos_sub = self._ros_node.create_subscription(
            Float32, "elbow_down", self.decreaseElbowPosition, 10
        )

    def getMotorPosition(self, motor_config: MoteusMotorConfig) -> float:
        position = self._info.getMotorState(motor_config).position
        position_radians = (position if position is not None else 0.0) * self.REVS_TO_RADIANS
        return position_radians

    def cwTurntablePosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_turntable_pos += 0.1
        self._interface.runMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR, self.target_turntable_pos)
    
    def ccwTurntablePosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_turntable_pos -= 0.1
        self._interface.runMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR, self.target_turntable_pos)

    def increaseShoulderPosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_shoulder_pos += 0.1
        self.checkShoulderTargetPos()
        self._interface.runMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR, self.target_shoulder_pos)

    def decreaseShoulderPosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_shoulder_pos -= 0.1
        self.checkShoulderTargetPos()
        self._interface.runMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR, self.target_shoulder_pos)

    def increaseElbowPosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_elbow_pos += 0.1
        self.checkElbowTargetPos()
        self._interface.runMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR, self.target_elbow_pos)

    def decreaseElbowPosition(self, msg: Float32) -> None:
        if not self.can_send:
            return
        self.target_elbow_pos -= 0.1
        self.checkElbowTargetPos()
        self._interface.runMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR, self.target_elbow_pos)

    def checkShoulderTargetPos(self) -> None:
        if self.target_shoulder_pos > self.MAX_SHOULDER_POS:
            self.target_shoulder_pos = self.MAX_SHOULDER_POS
        elif self.target_shoulder_pos < self.MIN_SHOULDER_POS:
            self.target_shoulder_pos = self.MIN_SHOULDER_POS

    def checkElbowTargetPos(self) -> None:
        if self.target_elbow_pos > self.MAX_ELBOW_POS:
            self.target_elbow_pos = self.MAX_ELBOW_POS
        elif self.target_elbow_pos < self.MIN_ELBOW_POS:
            self.target_elbow_pos = self.MIN_ELBOW_POS

    def setTargetPositions(self) -> None:
        # set initial target positions to motors' starting positions
        self.target_turntable_pos = self.getMotorPosition(MotorConfigs.ARM_TURNTABLE_MOTOR)
        self.target_shoulder_pos = self.getMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR)
        self.target_elbow_pos = self.getMotorPosition(MotorConfigs.ARM_ELBOW_MOTOR)