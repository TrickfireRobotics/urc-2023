import time

import can
from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MotorConfigs
from lib.interface.robot_info import RobotInfo
from lib.interface.robot_interface import RobotInterface


class IndividualControlVel:
    WRIST_VEL = 0.3
    VEL = 1.0

    def __init__(self, ros_node: Node, interface: RobotInterface, info: RobotInfo):
        self._ros_node = ros_node
        self.bot_interface = interface
        self.bot_info = info

        self.can_send = False

        # Elbow motor
        self.elbow_up_sub = ros_node.create_subscription(Float32, "elbow_up", self.elbowUp, 10)

        self.elbow_down_sub = ros_node.create_subscription(
            Float32, "elbow_down", self.elbowDown, 10
        )

        # Shoulder motor
        self.shoulder_up_sub = ros_node.create_subscription(
            Float32, "shoulder_up", self.shoulderUp, 10
        )

        self.elboshoulder_down_sub = ros_node.create_subscription(
            Float32, "shoulder_down", self.shoulderDown, 10
        )

        # Turn table
        self.turntable_clockwise_sub = ros_node.create_subscription(
            Float32, "turntable_cw", self.turntableCW, 10
        )

        self.turntable_counter_clockwise_sub = ros_node.create_subscription(
            Float32, "turntable_ccw", self.turntableCCW, 10
        )

        self.turntableMotorPos = 0
        self.turntableMotorSpeed = 0
        self.turntableMotorFaultCode = 0

        self.updateTurntableMotorState()

        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            msg = can.Message(
                arbitration_id=0x00000405,
                data=[0x00, 0x00, 0x00, 0x00],
                is_extended_id=True,
            )
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"MOVE Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")

    def updateTurntableMotorState(self, send_msg_before: bool = True) -> None:
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            if send_msg_before:
                msg = can.Message(
                    arbitration_id=0x00000305,  # velocity loop
                    data=[0x00, 0x00, 0x00, 0x00],
                    is_extended_id=True,
                )
                try:
                    bus.send(msg)
                    self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
                except can.CanError:
                    self._ros_node.get_logger().info("Message NOT sent")

            for msg in bus:
                self._ros_node.get_logger().info(f"{msg.arbitration_id:X}: {msg.data}")
                if msg.arbitration_id == 0x00002905:
                    self.turntableMotorPos = int(
                        self.uint8_to_bin(msg.data[0]) + self.uint8_to_bin(msg.data[1]), 2
                    )
                    self._ros_node.get_logger().info(
                        f"Turntable motor position: {self.turntableMotorPos}"
                    )
                    self.turntableMotorSpeed = int(
                        self.uint8_to_bin(msg.data[2]) + self.uint8_to_bin(msg.data[3]), 2
                    )
                    self._ros_node.get_logger().info(
                        f"Turntable motor speed {self.turntableMotorSpeed}"
                    )
                    self.turntableMotorFaultCode = msg.data[7]
                    if self.turntableMotorFaultCode != 0:
                        self._ros_node.get_logger().info(
                            f"TURNTABLE FAULT: {self.turntableMotorFaultCode}"
                        )

    def moveTurntableMotorLilBit(self) -> None:
        self._ros_node.get_logger().info("MOVING TURNTABLE POSITIVE")
        self.updateTurntableMotorState()
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            msg = can.Message(
                arbitration_id=0x00000305, data=[0x00, 0x00, 0x09, 0xC4], is_extended_id=True
            )  # help
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
                return

            time.sleep(0.5)

            msg2 = can.Message(
                arbitration_id=0x00000305, data=[0x00, 0x00, 0x00, 0x00], is_extended_id=True
            )

            try:
                bus.send(msg2)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
                return

    def moveTurntableMotorLilBitOtherWay(self) -> None:
        self._ros_node.get_logger().info("MOVING TURNTABLE NEGATIVE")
        self.updateTurntableMotorState()
        with can.Bus(interface="socketcan", channel="can0", receive_own_messages=True) as bus:
            msg = can.Message(
                arbitration_id=0x00000305, data=[0x00, 0x00, 0xF6, 0x3C], is_extended_id=True
            )  # help
            try:
                bus.send(msg)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
                return

            time.sleep(0.5)

            msg2 = can.Message(
                arbitration_id=0x00000305, data=[0x00, 0x00, 0x00, 0x00], is_extended_id=True
            )

            try:
                bus.send(msg2)
                self._ros_node.get_logger().info(f"Message sent on {bus.channel_info}")
            except can.CanError:
                self._ros_node.get_logger().info("Message NOT sent")
                return

    def uint8_to_bin(self, value: int) -> str:
        if 0 <= value <= 255:  # Ensure it's within uint8 range
            return format(value, "08b")  # Format as an 8-bit binary string
        else:
            raise ValueError("Value must be between 0 and 255")

    def elbowUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Elbow up" + str(data))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_ELBOW_MOTOR, -self.VEL)

        else:
            self._ros_node.get_logger().info("Elbow down STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)

    def elbowDown(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Elbow down" + str(data))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_ELBOW_MOTOR, self.VEL)

        else:
            self._ros_node.get_logger().info("Elbow down STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_ELBOW_MOTOR)

    def shoulderUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Shoulder up" + str(data))

            # this line uses velocity to run shoulder motor
            # i replace it with incrementing position because velocity on new shoulder motor scary
            # uncomment this line and comment out the other run motor line if it doesn't work
            # same with shoulderDown method
            # self.bot_interface.runMotorSpeed(MotorConfigs.ARM_SHOULDER_MOTOR, -self.VEL)

            motorState = self.bot_info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR)

            curPos = motorState.position
            if curPos == None:
                self._ros_node.get_logger().info(
                    "Couldn't get shoulder position. something is wrong..."
                )
                curPos = 0.0
            newPos = curPos + 0.02  # revolutions
            newPosRadians = newPos * 6.28  # radians
            self._ros_node.get_logger().info(
                f"Running shoulder to position (radians) {newPosRadians}"
            )
            self.bot_interface.runMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR, newPosRadians)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)

    def shoulderDown(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            motorState = self.bot_info.getMotorState(MotorConfigs.ARM_SHOULDER_MOTOR)

            curPos = motorState.position
            if curPos == None:
                self._ros_node.get_logger().info(
                    "Couldn't get shoulder position. something is wrong..."
                )
                curPos = 0.0
            newPos = curPos - 0.02  # revolutions
            newPosRadians = newPos * 6.28  # radians
            self._ros_node.get_logger().info(
                f"Running shoulder to position (radians) {newPosRadians}"
            )
            self.bot_interface.runMotorPosition(MotorConfigs.ARM_SHOULDER_MOTOR, newPosRadians)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)

    def turntableCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self.moveTurntableMotorLilBit()

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.updateTurntableMotorState()

    def turntableCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self.moveTurntableMotorLilBitOtherWay()

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.updateTurntableMotorState()
            self.updateTurntableMotorState()
            self.updateTurntableMotorState()
