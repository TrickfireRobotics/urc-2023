"""This example automatically connects to a Go Direct device via USB (if USB
is not connected, then it searches for the nearest GoDirect device via Bluetooth)
and starts reading measurements from the default sensor at a period of
1000ms (1 sample/second). Unlike the 'gdx_getting_started' examples that use the gdx module,
this example works directly with the godirect module. This example might
be important for troubleshooting, or if you do not want to use the gdx module.

If you want to enable specific sensors, you will need to know the sensor numbers.
Run the example called 'gdx_getting_started_device_info.py' to get that information.

Installation of the godirect package is required using 'pip3 install godirect'
"""

import logging
import sys
import threading

import rclpy  # Import the package
from godirect import GoDirect
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr  # Import yummy colors
from lib.configs import MotorConfigs
from lib.interface.robot_info import RobotInfo  # Read data
from lib.interface.robot_interface import RobotInterface  # Send data

# TRICKFIRE COMMENTS:
# A lot of this code is a part of the sample code from Vernier, take the comments with a grain a salt since we do not need to use a lot of the features provided with the device
# Im aware that there is a gdx version which is easier to use, but I could not get it working for some reason
# I recently added multithreading to allow for easy stopping and starting but i have not been able to do extensive testing on it yet
# The main plan is to show the difference between each measurement and to give the average difference over the entire measurement period (and hopefully be able to visualize it as a graph)


logging.basicConfig()


class GoDirectNode(Node):

    def __init__(self) -> None:
        self.avg_diff = 0.0
        self.example_pub = self.create_publisher(float, "science_topic", 10)
        self.example_pub_timer = self.create_timer(1.0, self.publish_data)

    def collect_data(self) -> None:
        # The first USB device found will be used. If no USB devices are found, then
        # the BLE device with the strongest signal over -100 is used.
        godirect = GoDirect(use_ble=False, use_usb=True)
        print("GoDirect v" + str(godirect.get_version()))
        print("\nSearching...", flush=True, end="")
        device = godirect.get_device(threshold=-100)
        stop_flag = False

        def get_user_input() -> None:
            nonlocal stop_flag
            input("Press Enter to stop...\n")
            stop_flag = True

        input_thread = threading.Thread(target=get_user_input, daemon=True)

        # Once a device is found or selected it must be opened.
        if device != None and device.open(auto_start=False):
            print("connecting.\n")
            print("Connected to " + device.name)

            # A specific sensor (or sensors) can be selected for data collection by calling
            # device.enable_sensors([]) prior to calling device.start().
            # If you do not use device.enable_sensors([]), the default sensor(s) will be automatically
            # enabled when device.start() is called.

            # Length is the amount of measurements to take, since one measurement is taken every second, this is also the amount of seconds the program will run for
            length = int(input("How Long? \n"))
            if length <= 0:
                length = 1

            # Set a baseline (preferably the normal avg difference in the area) the baseline will be removed from the total showing the true average difference
            baseline = float(input("Baseline? (input 0 if no)\n"))

            # The start period is how often it collects data in milliseconds, i would reccomend leaving it at 1000 for now as I experienced some bugs when changing the value
            device.start(period=1000)
            print("start")
            sensors = (
                device.get_enabled_sensors()
            )  # after start() is called, an enabled sensor list is available

            print("Reading measurements\n")
            input_thread.start()

            # Doing one check before for loop so that we can take difference easier
            # Keep in mind that since there can be multiple sensors and multiple values both values and sensors are vectors/arrays (idk)
            # but we are only using one sensor with one value so our info will be at 0
            if device.read():
                total = 0
                amt = sensors[0].values[0]
                print(sensors[0].sensor_description + ": " + str(sensors[0].values))
                sensors[0].clear()

            for i in range(1, length):
                if device.read():
                    if stop_flag:
                        break
                    for sensor in sensors:
                        # The 'sensor.values' call returns a list of measurements. This list might contain
                        # one sensor value, or multiple sensor values (if fast sampling)
                        print(sensor.sensor_description + ": " + str(sensor.values))
                        total += abs(amt - sensor.values[0])
                        amt = sensor.values[0]
                        self.publish_data(amt)
                        sensor.clear()
            device.stop()
            device.close()
            print("\nDisconnected from " + device.name)
            self.avg_diff = (total / (length / 2)) - baseline
            print("Avg diff: " + str(self.avg_diff))

        else:
            print("Go Direct device not found/opened")

        godirect.quit()

    def publish_data(self, datapoint: float = 0.0) -> None:
        if self.avg_diff is not None:
            msg = str(datapoint)
            msg.data = str(self.avg_diff)
            self.example_pub.publish(msg)
