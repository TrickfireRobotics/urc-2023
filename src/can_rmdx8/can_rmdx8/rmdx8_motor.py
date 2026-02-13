# Write the code for the node, implement everything that you can until
# the other RMD task is finished. Make sure to mimic the behaviour of
# the current Moteus node, but not necessarily copy the code.


import math
import time
from threading import Lock
from typing import TypeGuard

import myactuator_rmd_py as rmd
import std_msgs.msg
from myactuator_rmd_py.actuator_state import Gains
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr
from lib.configs import RMDx8MotorConfig
from lib.motor_state.rmd_motor_state import RMDX8MotorState, RMDX8RunSettings

DEGREE_TO_REV = 360


def _validOrZero(value: float | None) -> float:
    if not _checkValid(value):
        return 0.0
    return value


def _checkValid(value: float | None) -> TypeGuard[float]:
    if value is None or not math.isfinite(value):
        return False
    return True


class RMDx8Motor:
    """
    A wrapper class to interact with the RMDx8 actuator and the ROS nodes.
    """

    mutex_lock = Lock()

    def __init__(self, config: RMDx8MotorConfig, driver: rmd.CanDriver, ros_node: Node) -> None:

        self.config = config
        self._ros_node = ros_node
        self.motor = rmd.ActuatorInterface(driver, config.can_id)
        self._subscriber = self._createSubscriber()

        self._publisher = self._createPublisher()
        self._last_message_time = time.time()

    # create a subscriber
    def _createSubscriber(self) -> Subscription:
        topic_name = self.config.getInterfaceTopicName()
        subscriber = self._ros_node.create_subscription(
            std_msgs.msg.String,
            topic_name,
            self.dataInCallback,
            10,
        )
        return subscriber

    # create a publisher
    def _createPublisher(self) -> Publisher:
        """
        The publisher to send data to.
        """
        topic_name = self.config.getCanTopicName()
        # Size of queue is 1. All additional ones are dropped
        publisher = self._ros_node.create_publisher(std_msgs.msg.String, topic_name, 10)
        self._ros_node.get_logger().info("RMDx8 Publisher Created!!")
        return publisher

    def dataInCallback(self, msg: String) -> None:
        """
        Updates the RMDx8 motor state
        """
        run_settings = RMDX8RunSettings.fromJsonMsg(msg)
        if time.time() - self._last_message_time < 0.05:
            return
        self._last_message_time = time.time()

        try:
            with self.mutex_lock:
                # Stop if requested
                if run_settings.set_stop:
                    self._ros_node.get_logger().info("STOP")
                    self.stopMotor()
                    return

                if (
                    _checkValid(run_settings.position)
                    + _checkValid(run_settings.velocity)
                    + _checkValid(run_settings.current)
                    > 1
                ):
                    raise ValueError(
                        "Only one of `position`, `velocity`, or `current` can be present"
                    )

                # Else, set values
                if (
                    run_settings.speed_pi is not None
                    and run_settings.current_pi is not None
                    and run_settings.position_pi is not None
                ):
                    self.motor.setControllerGains(
                        Gains(
                            run_settings.current_pi, run_settings.speed_pi, run_settings.position_pi
                        )
                    )
                    self._ros_node.get_logger().info("SET GAINS")
                elif (
                    run_settings.speed_pi is not None
                    or run_settings.current_pi is not None
                    or run_settings.position_pi is not None
                ):
                    raise ValueError("All 3 PiGains must all be defined or all be None")

                if _checkValid(run_settings.position):
                    # Position is 0.01 dps and velocity is dps
                    self.motor.sendPositionAbsoluteSetpoint(
                        run_settings.position * DEGREE_TO_REV * 100,
                        _validOrZero(run_settings.velocity) * DEGREE_TO_REV,
                    )
                    self._ros_node.get_logger().info("POSITION")
                if _checkValid(run_settings.velocity):
                    # Velocity is 0.01 dps
                    self.motor.sendVelocitySetpoint(run_settings.velocity * DEGREE_TO_REV * 100)
                    self._ros_node.get_logger().info("VELOCITY")

                if _checkValid(run_settings.current):
                    # Value is 0.01 A
                    self.motor.sendCurrentSetpoint(run_settings.current * 100)
                    self._ros_node.get_logger().info("CURRENT")

                # Acceleration and type must both be set
                if _checkValid(run_settings.acceleration) != (
                    run_settings.acceleration_type is not None
                ):
                    raise ValueError(
                        "`acceleration` and `acceleration_type` must both be None or not None"
                    )
                if (
                    _checkValid(run_settings.acceleration)
                    and run_settings.acceleration_type is not None
                ):
                    # Acceleration is dps/s
                    self.motor.setAcceleration(
                        run_settings.acceleration * 360, run_settings.acceleration_type
                    )
                    self._ros_node.get_logger().info("ACCLEREATION")
        except (rmd.ProtocolException, rmd.can.ProtocolViolationError) as ex:
            self._ros_node.get_logger().warning(f"Ignoring rmd protocol exception: {ex}")

    def publishData(self) -> None:
        """
        Publishes data from the rmdx8 controller
        """
        try:
            with self.mutex_lock:
                state = RMDX8MotorState.fromRMDX8Data(
                    self.config.can_id,
                    self.motor.getMotorStatus1(),
                    self.motor.getMotorStatus2(),
                    self.motor.getMotorPower(),
                    self.motor.getAcceleration(),
                )
                if state.error_code != 0:
                    self.motor.shutdownMotor()
                    self._ros_node.get_logger().error(
                        colorStr(
                            "RMDx8 Motor Error State: " + str(state.error_code), ColorCodes.FAIL_RED
                        )
                    )
            self._publisher.publish(state.toMsg())
        except (
            rmd.ProtocolException,
            rmd.can.ProtocolViolationError,
            rmd.can.SocketException,
        ) as ex:
            self._ros_node.get_logger().info("Ignoring socket exception: " + str(ex))

    def stopMotor(self) -> None:
        """
        Calls my_actuator_rmd stopMotor
        """
        with self.mutex_lock:
            self.motor.stopMotor()

    def shutdownMotor(self) -> None:
        """
        Calls my_actuator_rmd shutdownMotor
        """
        with self.mutex_lock:
            self.motor.shutdownMotor()
