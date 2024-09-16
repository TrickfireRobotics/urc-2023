import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from usb.core import find as finddev

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs

from . import moteus_thread_manager


class RosMotuesBridge(Node):
    """
    This is the node that connects the Moteus Motor Controllers
    with the rest of the ROS codebase. You can easily add new
    motors using the addMotor() function from the self.threadManager.

    You can attempt to reconnect to the Moteus Controllers during runtime.
    The CANFD-USB is reset every time the code is execute from the ./launch.sh
    """

    def __init__(self) -> None:
        super().__init__("can_moteus_node")
        self.get_logger().info(colorStr("Launching can_moteus node", ColorCodes.BLUE_OK))

        # Reset the CANFD-USB
        # run "lsusb" in cmd with the CANFD-USB connected
        # to find the idVendor and the idProduct
        dev = finddev(idVendor=0x0483, idProduct=0x5740)

        if dev is not None:
            dev.reset()
            self.thread_manager: moteus_thread_manager.MoteusThreadManager | None = None

            self.reconnect_to_moteus_sub = self.create_subscription(
                Float32, "reconnectMoteusControllers", self.reconnect, 1
            )

            self.createMoteusMotors()
        else:
            self.get_logger().info(
                colorStr(
                    "Failed to find CANFD-USB usb device. Is it plugged in?", ColorCodes.FAIL_RED
                )
            )

    def reconnect(self, _: Float32) -> None:
        """
        Gracefully shuts down the threadManager and creates a new instance of
        the threadManager object.

        """
        self.get_logger().info("Reconnecting")
        if self.thread_manager is not None:
            self.thread_manager.reconnectMotors()

    def createMoteusMotors(self) -> None:
        """
        Creates the threadManager and adds all the moteus motors
        """

        self.thread_manager = moteus_thread_manager.MoteusThreadManager(self)

        # Drivebase
        self.thread_manager.addMotor(MotorConfigs.REAR_RIGHT_DRIVE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.MID_RIGHT_DRIVE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.FRONT_RIGHT_DRIVE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.REAR_LEFT_DRIVE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.MID_LEFT_DRIVE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.FRONT_LEFT_DRIVE_MOTOR)

        # Arm
        self.thread_manager.addMotor(MotorConfigs.ARM_TURNTABLE_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.ARM_SHOULDER_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.ARM_ELBOW_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.ARM_LEFT_WRIST_MOTOR)
        self.thread_manager.addMotor(MotorConfigs.ARM_RIGHT_WRIST_MOTOR)

        self.thread_manager.start()


def main(args: list[str] | None = None) -> None:
    """
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
        node.get_logger().info(colorStr("Shutting down can_moteus", ColorCodes.BLUE_OK))
        if node.thread_manager is not None:
            node.thread_manager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
