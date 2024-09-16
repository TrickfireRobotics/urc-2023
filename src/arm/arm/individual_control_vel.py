from rclpy.node import Node
from std_msgs.msg import Float32

from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface


class IndividualControlVel:
    WRIST_VEL = 0.3
    VEL = 1.0

    def __init__(self, ros_node: Node, interface: RobotInterface):
        self._ros_node = ros_node
        self.bot_interface = interface

        self.can_send = False

        # Left wrist motor
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_cw", self.leftWristCW, 10
        )
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_ccw", self.leftWristCCW, 10
        )

        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_cw", self.leftWristCW, 10
        )
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "left_wrist_ccw", self.leftWristCCW, 10
        )

        # Right wrist motor
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "right_wrist_cw", self.rightWristCW, 10
        )
        self.left_wrist_motor_sub = ros_node.create_subscription(
            Float32, "right_wrist_ccw", self.rightWristCCW, 10
        )

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

    def leftWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        self.turntable_counter_clockwise_sub = ros_node.create_subscription(
            Float32, "turntable_ccw", self.turntableCCW, 10
        )

    def leftWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Left Wrist CW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_LEFT_WRIST_MOTOR, self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)

    def leftWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Left Wrist CCW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_LEFT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)

    def rightWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_RIGHT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

    def rightWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CCW" + str(joystick))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_RIGHT_WRIST_MOTOR, -self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

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
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_SHOULDER_MOTOR, -self.VEL)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)

    def shoulderDown(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Shoulder down" + str(data))
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_SHOULDER_MOTOR, self.VEL)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_SHOULDER_MOTOR)

    def turntableCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Turntable clock wise")
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_TURNTABLE_MOTOR, self.VEL)

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)

    def turntableCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

    def turntableCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Turntable counter clock wise")
            self.bot_interface.runMotorSpeed(MotorConfigs.ARM_TURNTABLE_MOTOR, -self.VEL)

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.stopMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)
