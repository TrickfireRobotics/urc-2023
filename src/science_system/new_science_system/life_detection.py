""" This example automatically connects to a Go Direct device via USB (if USB 
is not connected, then it searches for the nearest GoDirect device via Bluetooth)
and starts reading measurements from the default sensor at a period of 
1000ms (1 sample/second). Unlike the 'gdx_getting_started' examples that use the gdx module,
this example works directly with the godirect module. This example might
be important for troubleshooting, or if you do not want to use the gdx module.

If you want to enable specific sensors, you will need to know the sensor numbers.
Run the example called 'gdx_getting_started_device_info.py' to get that information.

Installation of the godirect package is required using 'pip3 install godirect'
"""

from godirect import GoDirect

# TRICKFIRE COMMENTS:
# A lot of this code is a part of the sample code from Vernier, take the comments with a grain a salt since we do not need to use a lot of the features provided with the device
# Im aware that there is a gdx version which is easier to use, but I could not get it working for some reason
# I recently added multithreading to allow for easy stopping and starting but i have not been able to do extensive testing on it yet
# I also recently made some changes to make it compatible with the repository so I hope nothing broke, since I have not been able to test it yet
# The main plan is to show the difference between each measurement and to give the average difference over the entire measurement period (and hopefully be able to visualize it as a graph)


import logging
import threading
logging.basicConfig()

stop_flag = False

def get_user_input() -> None:
    global stop_flag
    input("Press Enter to stop...\n")
    stop_flag = True

input_thread = threading.Thread(target=get_user_input, daemon=True)

def main(args: list[str] | None = None) -> None:
        # The first USB device found will be used. If no USB devices are found, then 
        # the BLE device with the strongest signal over -100 is used.
        godirect = GoDirect(use_ble=False, use_usb=True)
        print("GoDirect v"+str(godirect.get_version()))
        print("\nSearching...", flush=True, end ="")
        device = godirect.get_device(threshold=-100)

        # Once a device is found or selected it must be opened.
        if device != None and device.open(auto_start=False):
                print("connecting.\n")
                print("Connected to "+device.name)

                # A specific sensor (or sensors) can be selected for data collection by calling
                # device.enable_sensors([]) prior to calling device.start().
                # If you do not use device.enable_sensors([]), the default sensor(s) will be automatically
                # enabled when device.start() is called. 

                # The start period is how often it collects data in milliseconds, i would reccomend leaving it at 1000 for now as I experienced some bugs when changing the value
                device.start(period=1000) 
                print("start")
                sensors = device.get_enabled_sensors()   # after start() is called, an enabled sensor list is available
                
                # Lenght is the amount of measurements to take, since one measurement is taken every second, this is also the amount of seconds the program will run for
                print("How Long?")
                length = int(input())

                print("Reading measurements\n")
                input_thread.start()
                for i in range(0,length):
                        if device.read():
                                if (stop_flag):
                                        break  
                                for sensor in sensors:
                                        # The 'sensor.values' call returns a list of measurements. This list might contain 
                                        # one sensor value, or multiple sensor values (if fast sampling)
                                        print(sensor.sensor_description+": "+str(sensor.values))                
                                        sensor.clear()
                device.stop()
                device.close()
                print("\nDisconnected from "+device.name)

        else:
                print("Go Direct device not found/opened")

        godirect.quit()