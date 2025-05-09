import time

from arduino import ArduinoNanoEvery
from i2c_bus import I2CBus


def main():
    bus = I2CBus()
    arduino = ArduinoNanoEvery(bus)

    # one-time scan for debugging
    print("I2C devices:", [hex(a) for a in bus.scan])

    while True:
        arduino.send_command(0x01)
        print("Sent ping -> Arduino")
        time.sleep(1)


if __name__ == "__main__":
    main()
