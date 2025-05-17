# Write the code for the node, implement everything that you can until
# the other RMD task is finished. Make sure to mimic the behaviour of
# the current Moteus node, but not necessarily copy the code.


import math
from threading import Lock

import myactuator_rmd_py as rmd
import std_msgs.msg
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.configs import RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8MotorState, RMDX8RunSettings


class RMDx8Motor:
    """
    A wrapper class to interact with the RMDx8 actuator and the ROS nodes.
    """

    mutex_lock = Lock()

    def __init__(self, config: RMDx8MotorConfig, driver: rmd.CanDriver, ros_node: Node) -> None:

        self.config = config
        self._ros_node = ros_node
        self.my_actuator = rmd.ActuatorInterface(driver, config.can_id)
        self._subscriber = self._createSubscriber()

        self._publisher = self._createPublisher()

        # Publish a message every 0.05 seconds
        timer_period = 0.05
        self.timer = ros_node.create_timer(timer_period, self.publishData)

    # create a subscriber
    def _createSubscriber(self) -> Subscription:
        topic_name = self.config.getInterfaceTopicName()
        subscriber = self._ros_node.create_subscription(
            std_msgs.msg.String,
            topic_name,
            self.dataInCallback,
            1,
        )
        return subscriber

    # create a publisher
    def _createPublisher(self) -> Publisher:
        """
        The publisher to send data to.
        """
        topic_name = self.config.getCanTopicName()
        # Size of queue is 1. All additional ones are dropped
        publisher = self._ros_node.create_publisher(std_msgs.msg.String, topic_name, 1)
        self._ros_node.get_logger().info("RMDx8 Publisher Created!!")
        return publisher

    def dataInCallback(self, msg: String) -> None:
        """
        Updates the RMDx8 motor state
        """
        run_settings = RMDX8RunSettings.fromJsonMsg(msg)

        with self.mutex_lock:
            # Stop if requested
            if run_settings.set_stop:
                self.my_actuator.stopMotor()
            else:
                # Else, set position and velocity if they exist
                if run_settings.position is not None and math.isfinite(run_settings.position):
                    self.my_actuator.sendPositionAbsoluteSetpoint(
                        run_settings.position,
                        0.0 if run_settings.velocity is None else run_settings.velocity_limit,
                    )
                if run_settings.velocity is not None and math.isfinite(run_settings.velocity):
                    self.my_actuator.sendVelocitySetpoint(run_settings.velocity)

    def publishData(self) -> None:
        """
        Publishes data from the rmdx8 controller
        """
        with self.mutex_lock:
            state = RMDX8MotorState.fromRMDX8Data(
                self.config.can_id,
                self.my_actuator.getMotorStatus1(),
                self.my_actuator.getMotorStatus2(),
                self.my_actuator.getMotorPower(),
                self.my_actuator.getAcceleration(),
            )
        self._publisher.publish(state.toMsg())

    def stopMotor(self) -> None:
        """
        Calls my_actuator_rmd stopMotor
        """
        with self.mutex_lock:
            self.my_actuator.stopMotor()

    def shutdownMotor(self) -> None:
        """
        Calls my_actuator_rmd shutdownMotor
        """
        with self.mutex_lock:
            self.my_actuator.shutdownMotor()
