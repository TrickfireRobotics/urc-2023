#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

import sys
sys.path.append("/home/trickfire/urc-2023/src")

from lib.color_codes import ColorCodes, colorStr

from lib.interface.robot_interface import RobotInterface
from lib.interface.robot_info import RobotInfo
from lib import configs

# Credit: Most of this code is credit to Anna. I (Hong) just
# add some finishing code and clean up the class.

class Heartbeat(Node):
    """
    The Heartbeat is responsible to check the connection status
    of the rover and shut down all motors if the connection is lost.

    Args:
        Node (Node): ROS Node
    """

    def __init__(self):
        """
        Initialize the heartbeat node and provides it with a timer
        to keep track of the connection status of the rover.
        """

        # initialize heartbeat node
        super().__init__("heartbeat_node")

        # print node name [debug purpose]
        self.get_logger().info(colorStr("Launching heartbeat_node", ColorCodes.BLUE_OK))

        # robot interface: send data
        # robot info: retrieve data
        # self._robot_info = RobotInfo(self)
        self._robot_interface = RobotInterface(self)

        # give enough time (10s) for others to initialize
        time.sleep(10)

        # flag to store connection status
        self._connection_lost = False

        # create subscription to mission control
        self._is_alive_subscriber = self.create_subscription(
            Bool, # message type (might change later)
            "/heartbeat", # topic name
            self.heartbeat_callback, # callback function
            10  # Quality of Service (QoS) profile
        )

        # store the most recent hearbeat time
        self._last_heartbeat_time = time.time()

        # check connection every 1 second
        self._timer = self.create_timer(1.0, self.check_connection) 

    

    def heartbeat_callback(self, msg):
        """
        A call back method for the heartbeat everytime the
        mission control publish a message indicating that the
        connection is still active.

        Args:
            msg (Bool): a ROS message type retrieved from the
                        publisher that indicates the connection
                        status.
        """

        # self.get_logger().info("heartbeat call back")

        # Update the timestamp of the last received heartbeat message
        self._last_heartbeat_time = time.time()

        # Log connection active as before
        # doesn't matter the data, pub always pub True
        # just check to make sure nothing is wrong with pub
        if msg.data == True:
            self.get_logger().info(colorStr("Connection active", ColorCodes.GREEN_OK))
            self._connection_lost = False


    def check_connection(self):
        """
        A method for checking the connection status. If the heartbeat
        does not receive a message within 1s, meaning the connection was
        lost, then it will shut down all motors.
        """

        # Check if the last heartbeat was received more than a threshold ago
        if time.time() - self._last_heartbeat_time > 2:  # threshold = 2 seconds
            # if already shut down motor, don't have to do this again
            if self._connection_lost:
                return

            self.get_logger().warning(colorStr("Connection lost", ColorCodes.WARNING_YELLOW))
            self._connection_lost = True

            # call the robot interface to stop all motors
            self._stop_all_motors()

    
    # ***************
    # Private helper method
    # ***************
    def _stop_all_motors(self):
        """
        Get all motors from the motor configs file and stop all of them.
        """

        # get all the motors from configs.py
        all_motors = configs.MotorConfigs.getAllMotors()

        # loop through each motor and stop it via the robot interface
        for motor in all_motors:
            self._robot_interface.stopMotor(motor)

            # debug message
            self.get_logger().info(colorStr("Stop motor can id" + str(motor.can_id), ColorCodes.FAIL_RED))

        
        # finish [debug message]
        self.get_logger().info(colorStr("Stop all motors!", ColorCodes.FAIL_RED))


def main(args=None):
    rclpy.init(args=args)
    heartbeat_node = Heartbeat()
    rclpy.spin(heartbeat_node)

    heartbeat_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    Heartbeat.main()