from rclpy import Node
from std_msgs.msg import Float32

from interface.robot_interface import RobotInterface


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
        self.elbow_up_sub = ros_node.create_subscription(
            Float32, "elbow_up", self.elbowUp, 10
        )

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
            self.bot_interface.leftWriteVelocity(self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.leftWriteVelocity(0.0)

    def leftWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Left Wrist CCW" + str(joystick))
            self.bot_interface.leftWriteVelocity(-self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Left Wrist STOP")
            self.bot_interface.leftWriteVelocity(0.0)

    def rightWristCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CW" + str(joystick))
            self.bot_interface.rightWristVelocity(-self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.rightWristVelocity(0.0)

    def rightWristCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        joystick = msg.data

        if joystick > 0:
            self._ros_node.get_logger().info("Right Wrist CCW" + str(joystick))
            self.bot_interface.rightWristVelocity(-self.WRIST_VEL)

        else:
            self._ros_node.get_logger().info("Right Wrist STOP")
            self.bot_interface.rightWristVelocity(0.0)

    def elbowUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Elbow up" + str(data))
            self.bot_interface.elbowVelocity(-self.VEL)

        else:
            self._ros_node.get_logger().info("Elbow down STOP")
            self.bot_interface.elbowVelocity(0.0)

    def elbowDown(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Elbow down" + str(data))
            self.bot_interface.elbowVelocity(self.VEL)

        else:
            self._ros_node.get_logger().info("Elbow down STOP")
            self.bot_interface.elbowVelocity(0.0)

    def shoulderUp(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Shoulder up" + str(data))
            self.bot_interface.shoulderVelocity(-self.VEL)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.shoulderVelocity(0.0)

    def shoulderDown(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Shoulder down" + str(data))
            self.bot_interface.shoulderVelocity(self.VEL)

        else:
            self._ros_node.get_logger().info("Shoulder STOP")
            self.bot_interface.shoulderVelocity(0.0)

    def turntableCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Turntable clock wise")
            self.bot_interface.armTurntableVelocity(self.VEL)

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.armTurntableVelocity(0.0)

    def turntableCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

    def turntableCCW(self, msg: Float32) -> None:
        if not self.can_send:
            return

        data = msg.data

        if data > 0:
            self._ros_node.get_logger().info("Turntable counter clock wise")
            self.bot_interface.armTurntableVelocity(-self.VEL)

        else:
            self._ros_node.get_logger().info("Turntable STOP")
            self.bot_interface.armTurntableVelocity(0.0)
