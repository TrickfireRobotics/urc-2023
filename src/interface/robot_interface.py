# General interface for robot funcitonality

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float32, String

from utility.moteus_data_in_json_helper import MoteusDataInJsonHelper

VELOCITY_CONVERSION = 1
POSITION_CONVERSION = 1 / (6.28)  # Radians to position ticks
REV_TO_RADIANS = 6.28  # One Revolution = 2pi


robot_publishers: dict[str, Publisher] = {}


class RobotInterface:
    #
    # Constructors/Destructors
    #

    def __init__(self, ros_node: Node) -> None:
        self._ros_node = ros_node

        # Drive base
        publisher = self._ros_node.create_publisher(
            String, "front_left_drive_motor_from_interface", 10
        )
        robot_publishers["front_left_drive_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "front_right_drive_motor_from_interface", 10
        )
        robot_publishers["front_right_drive_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "mid_left_drive_motor_from_interface", 10
        )
        robot_publishers["mid_left_drive_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "mid_right_drive_motor_from_interface", 10
        )
        robot_publishers["mid_right_drive_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "rear_left_drive_motor_from_interface", 10
        )
        robot_publishers["rear_left_drive_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "rear_right_drive_motor_from_interface", 10
        )
        robot_publishers["rear_right_drive_motor"] = publisher

        robot_publishers["arm_fingers_motor"] = publisher

        # Antenna
        publisher = self._ros_node.create_publisher(
            Float32, "antenna_motor_position_from_interface", 10
        )
        robot_publishers["antenna_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            Float32, "antenna_turntable_motor_position_from_interface", 10
        )
        robot_publishers["antenna_turntable_motor"] = publisher

        # Arm
        publisher = self._ros_node.create_publisher(
            String, "arm_turntable_motor_from_interface", 10
        )
        robot_publishers["arm_turntable_motor"] = publisher
        publisher = self._ros_node.create_publisher(String, "arm_shoulder_motor_from_interface", 10)
        robot_publishers["arm_shoulder_motor"] = publisher
        publisher = self._ros_node.create_publisher(String, "arm_elbow_motor_from_interface", 10)
        robot_publishers["arm_elbow_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "arm_left_wrist_motor_from_interface", 10
        )
        robot_publishers["arm_left_wrist_motor"] = publisher
        publisher = self._ros_node.create_publisher(
            String, "arm_right_wrist_motor_from_interface", 10
        )
        robot_publishers["arm_right_wrist_motor"] = publisher

    def velocityConversion(self, amount: float) -> float:
        return amount * VELOCITY_CONVERSION

    # Converting from input "radians" to output "revolutions"
    def positionConversion(self, amount: float) -> float:
        return amount * POSITION_CONVERSION

    # General movement including all wheels
    def moveForward(self, amount: float) -> None:
        print()

    def moveBackward(self, amount: float) -> None:
        print()

    def turnLeft(self, amount: float) -> None:
        print()

    def turnRight(self, amount: float) -> None:
        print()

    # ------ DRIVEBASE ------
    # Left front wheel - MEASURED IN REVOLUTIONS PER SECOND
    def frontLeftDriveMotor(self, amount: float) -> None:
        self._sendDriveMotor("front_left_drive_motor", radians_per_second=amount)

    # Right front wheel
    def frontRightDriverMotor(self, amount: float) -> None:
        self._sendDriveMotor("front_right_drive_motor", radians_per_second=amount)

    # Left middle wheel
    def midLeftDriveMotor(self, amount: float) -> None:
        self._sendDriveMotor("mid_left_drive_motor", radians_per_second=amount)

    # Right middle wheel
    def midRightDriveMotor(self, amount: float) -> None:
        self._sendDriveMotor("mid_right_drive_motor", radians_per_second=amount)

    # Left back wheel
    def rearLeftDriveMotor(self, amount: float) -> None:
        self._sendDriveMotor("rear_left_drive_motor", radians_per_second=amount)

    # Right back wheel
    def rearRightDriveMotor(self, amount: float) -> None:
        self._sendDriveMotor("rear_right_drive_motor", radians_per_second=amount)

    def _sendDriveMotor(self, pub_name: str, radians_per_second: float = 0.0) -> None:
        publisher = robot_publishers[pub_name]

        str_msg = String()
        json_helper = MoteusDataInJsonHelper()
        json_helper.velocity = radians_per_second
        json_helper.set_stop = False

        str_msg.data = json_helper.buildJSONString()
        publisher.publish(str_msg)

    # ------ ARM ------

    def armTurntableVelocity(self, target_vel: float) -> None:
        self._sendArmVelocity("arm_turntable_motor", target_vel)

    def shoulderVelocity(self, target_vel: float) -> None:
        self._sendArmVelocity("arm_shoulder_motor", target_vel)

    def elbowVelocity(self, target_vel: float) -> None:
        self._sendArmVelocity("arm_elbow_motor", target_vel)

    def leftWriteVelocity(self, target_vel: float) -> None:
        self._sendArmVelocity("arm_left_wrist_motor", target_vel)

    def rightWristVelocity(self, target_vel: float) -> None:
        self._sendArmVelocity("arm_right_wrist_motor", target_vel)

    # For now, I am sending rev/sec
    def _sendArmVelocity(self, pub_name: str, velocity: float = 0.0) -> None:
        publisher = robot_publishers[pub_name]

        str_msg = String()
        json_helper = MoteusDataInJsonHelper()
        json_helper.velocity = velocity
        json_helper.set_stop = False

        str_msg.data = json_helper.buildJSONString()
        publisher.publish(str_msg)

    def disableArmTurntableMotor(self) -> None:
        self._disableMotor("arm_turntable_motor")

    def disableArmShoulderMotor(self) -> None:
        self._disableMotor("arm_shoulder_motor")

    def disableArmElbowMotor(self) -> None:
        self._disableMotor("arm_elbow_motor")

    def disableArmLeftWristMotor(self) -> None:
        self._disableMotor("arm_left_wrist_motor")

    def disableArmRightWristMotor(self) -> None:
        self._disableMotor("arm_right_wrist_motor")

    def _disableMotor(self, pub_name: str) -> None:
        publisher = robot_publishers[pub_name]

        str_msg = String()
        json_helper = MoteusDataInJsonHelper()
        json_helper.velocity = 0  # for extra security
        json_helper.set_stop = True

        str_msg.data = json_helper.buildJSONString()
        publisher.publish(str_msg)

    # Antenna movement
    def antennaMotorExtend(self) -> None:
        # Output is in "revolutions"
        revolutions_output = (
            1  # we don't know the actual position it should be. this is a placeholder
        )
        publisher = robot_publishers["antenna_motor"]
        str_msg = Float32()
        str_msg.data = revolutions_output
        publisher.publish(str_msg)

    def antennaMotorRetract(self) -> None:
        # Output is in "revolutions"
        revolutions_output = (
            0  # we don't know the actual position it should be. this is a placeholder
        )
        publisher = robot_publishers["antenna_motor"]
        str_msg = Float32()
        str_msg.data = revolutions_output
        publisher.publish(str_msg)

    def antennaTurntableMotor(self, amount: float) -> None:
        revolutions_output = self.positionConversion(amount)
        publisher = robot_publishers["antenna_turntable_motor"]
        str_msg = Float32()
        str_msg.data = revolutions_output
        publisher.publish(str_msg)
