import sys
from typing import Any

import rclpy
import serial
import serial.tools.list_ports
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# from lib.color_codes import ColorCodes, colorStr


class MotherboardDriver(Node):

    def __init__(self) -> None:
        super().__init__("mbd_node")
        self._ser: serial.Serial | None = None
        # self.get_logger().info(colorStr("Launching mbd_node node", ColorCodes.BLUE_OK))

    # def print_test(self) -> None:
    # self.get_logger().info(colorStr("HELLO WORLD!", ColorCodes.BLUE_OK))

    def find_pico_port(self) -> str | None:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"Port description: {port.description}")
            if "Pico" in port.description or "USB Serial Device" in port.description:
                print(f"Found Pico on {port.device}")
                return port.device
        print("Pico not found.")
        return None

    def test_serial(self) -> None:
        """
        Function to test the serial connection to the Pico MCU
        """

        port = self.find_pico_port()
        if port:
            self._ser = serial.Serial(port, 115200, timeout=1)
            print("Connected to pico")
        else:
            print("Unable to connect to pico")

        if self._ser is not None:
            try:
                while True:
                    msg = input("Enter a message to send: ")
                    self._ser.write((msg + "\n").encode("utf-8"))

                    response = self._ser.readLine().decode("utf-8").strip()
                    print(f"Received from pico: {response}")
            except KeyboardInterrupt:
                print("Closing Serial Connection.")
            finally:
                self._ser.close()
        else:
            print("Serial connection is not established.")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        mbd = MotherboardDriver()
        # mbd.print_test()
        mbd.test_serial()
        rclpy.spin(mbd)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # mbd.get_logger().info(colorStr("Shutting down mbd_node node", ColorCodes.BLUE_OK))
        mbd.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
