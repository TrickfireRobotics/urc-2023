import sys


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.node import Node
from std_msgs.msg import Float32
from usb.core import find as finddev

from utility.canbus_mappings import CanBusMappings
from utility.color_text import ColorCodes

from . import moteus_thread_manager


class RosMotuesBridge(Node):
    """
    This is the node that connects the Moteus Motor Controllers
    with the rest of the ROS codebase. You can easily add new
    motors using the addMotor() function from the self.threadManager.

    You can attempt to reconnect to the Moteus Controllers during runtime.
    The CANFD-USB is reset every time the code is execute from the ./launch.sh
    This is the node that connects the Moteus Motor Controllers
    with the rest of the ROS codebase. You can easily add new
    motors using the addMotor() function from the self.threadManager.

    You can attempt to reconnect to the Moteus Controllers during runtime.
    The CANFD-USB is reset every time the code is execute from the ./launch.sh
    """

    def __init__(self) -> None:

    def __init__(self) -> None:
        super().__init__("can_moteus_node")
        self.get_logger().info(
            ColorCodes.BLUE_OK + "Launching can_moteus node" + ColorCodes.ENDC
        )

        # Reset the CANFD-USB
        # run "lsusb" in cmd with the CANFD-USB connected
        # to find the idVendor and the idProduct
        dev = finddev(idVendor=0x0483, idProduct=0x5740)

        if dev is not None:
        if dev is not None:
            dev.reset()
            self.thread_manager: moteus_thread_manager.MoteusThreadManager | None = None
            self.canbus_mappings = CanBusMappings()

            self.reconnect_to_moteus_sub = self.create_subscription(
                Float32, "reconnectMoteusControllers", self.reconnect, 1
            )

            self.createMoteusMotors()
        else:
            self.get_logger().info(
                ColorCodes.FAIL_RED
                + "Failed to find CANFD-USB usb device. Is it plugged in?"
                + ColorCodes.ENDC
            )

    def reconnect(self, _: Float32) -> None:
        """
        Gracefully shuts down the threadManager and creates a new instance of
        the threadManager object.

        Gracefully shuts down the threadManager and creates a new instance of
        the threadManager object.

        """
        self.get_logger().info("Reconnecting")
        if self.thread_manager is not None:
            self.thread_manager.reconnectMotors()
        if self.thread_manager is not None:
            self.thread_manager.reconnectMotors()

    def createMoteusMotors(self) -> None:
    def createMoteusMotors(self) -> None:
        """
        Creates the threadManager and adds all the moteus motors
        Creates the threadManager and adds all the moteus motors
        """

        self.thread_manager = moteus_thread_manager.MoteusThreadManager(self)


        self.thread_manager = moteus_thread_manager.MoteusThreadManager(self)

        # Drivebase
        self.thread_manager.addMotor(
            CanBusMappings.CANID_REAR_RIGHT_DRIVE_MOTOR, "rear_right_drive_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_MID_RIGHT_DRIVE_MOTOR, "mid_right_drive_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_FRONT_RIGHT_DRIVE_MOTOR, "front_right_drive_motor"
        )

        self.thread_manager.addMotor(
            CanBusMappings.CANID_REAR_LEFT_DRIVE_MOTOR, "rear_left_drive_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_MID_LEFT_DRIVE_MOTOR, "mid_left_drive_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_FRONT_LEFT_DRIVE_MOTOR, "front_left_drive_motor"
        )

        # Arm
        self.thread_manager.addMotor(
            CanBusMappings.CANID_ARM_TURNTABLE_MOTOR, "arm_turntable_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_ARM_SHOULDER_MOTOR, "arm_shoulder_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_ARM_ELBOW_MOTOR, "arm_elbow_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_ARM_LEFT_WRIST_MOTOR, "arm_left_wrist_motor"
        )
        self.thread_manager.addMotor(
            CanBusMappings.CANID_ARM_RIGHT_WRIST_MOTOR, "arm_right_wrist_motor"
        )

        self.thread_manager.start()


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    The entry point of the node.
    """


    rclpy.init(args=args)
    try:
        node = RosMotuesBridge()
        rclpy.spin(node)


    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(ColorCodes.BLUE_OK + "Shutting down can_moteus" + ColorCodes.ENDC)
        if node.thread_manager is not None:
            node.thread_manager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)



if __name__ == "__main__":
    main()
