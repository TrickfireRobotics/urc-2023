import json
import sys

import rclpy
import RPi.GPIO as GPIO
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface

RED_PIN_NUM = 32  # GPIO32, physical pin 13
GREEN_PIN_NUM = 27  # GPIO27, physical pin 15
BLUE_PIN_NUM = 8  # GPIO08, physical pin16


class TempLight(Node):
    def __init__(self) -> None:
        super().__init__("temp_light_node")
        self.get_logger().info(colorStr("Launching temp_light node", ColorCodes.BLUE_OK))

        self._temp_light_subscriber = self.create_subscription(
            String, "temp_light_from_interface", self._handle_temp_light, 10
        )

        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
        GPIO.setup(RED_PIN_NUM, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(GREEN_PIN_NUM, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(BLUE_PIN_NUM, GPIO.OUT, initial=GPIO.HIGH)

        # self.bot_interface = RobotInterface(self)

        # self.timer = self.create_timer(0.5, self.timer_callback)

    def _handle_temp_light(self, msg: String) -> None:
        light_result = json.loads(msg.data)
        state = light_result["state"]

        # LOW all pins
        GPIO.output(RED_PIN_NUM, GPIO.LOW)
        GPIO.output(GREEN_PIN_NUM, GPIO.LOW)
        GPIO.output(BLUE_PIN_NUM, GPIO.LOW)

        if light_result["target"] == "RED":
            GPIO.output(RED_PIN_NUM, state)
        elif light_result["target"] == "GREEN":
            GPIO.output(GREEN_PIN_NUM, state)
        elif light_result["target"] == "BLUE":
            GPIO.output(BLUE_PIN_NUM, state)

    def timer_callback(self) -> None:
        self.bot_interface.setTempLight("GREEN", 0)


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        node = TempLight()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(colorStr("Shutting down temp_light", ColorCodes.BLUE_OK))
        if node.thread_manager is not None:
            node.thread_manager.terminateAllThreads()
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
