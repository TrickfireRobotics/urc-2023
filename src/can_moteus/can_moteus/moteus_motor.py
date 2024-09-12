from threading import Lock
from typing import Any

import moteus
import std_msgs.msg
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String

from lib.configs import MoteusMotorConfig
from lib.moteus_data_in_json_helper import MoteusDataInJsonHelper
from lib.moteus_data_out_json_helper import MoteusDataOutJsonHelper


class MoteusMotor:

    def __init__(self, config: MoteusMotorConfig, ros_node: Node) -> None:
        """
        Create a logical representation of a motor that is using
        a Moteus controller. Contains a list of variables that should be
        sent to the Moteus controller. Subscribes to input and publishes output

        Paramaters
        -------
        config : MoteusMotorConfig
            The config of the motor.
        rosNode : Node
            The ROS node used to create the ros_moteus_bridge.py
        """

        self.config = config
        self._ros_node = ros_node

        self._subscriber = self._createSubscriber()
        self._publisher = self._createPublisher()

        # Mutex - used to protect writing/reading the state of the motor
        self.mutex_lock = Lock()

        # The settings we can send to the moteus controller
        # using the make_position() method
        self.position: float | None = None
        self.velocity: float | None = None
        self.feedforward_torque: float | None = None
        self.kp_scale: float | None = None
        self.kd_scale: float | None = None
        self.max_torque: float | None = None
        self.watchdog_timeout: float | None = None
        self.velocity_limit: float | None = None
        self.accel_limit: float | None = None
        self.fixed_voltage_override: float | None = None

        # By default, the motor is shutoff
        self.set_stop = True

    def _createSubscriber(self) -> Subscription:
        """
        The subscriber to get data from.
        The format of the topic is the following: moteusmotor_<can_id>_from_interface
        """
        topic_name = self.config.getInterfaceTopicName()
        subscriber = self._ros_node.create_subscription(
            std_msgs.msg.String,
            topic_name,
            self.dataInCallback,
            1,  # Size of queue is 1. All additional ones are dropped
        )

        return subscriber

    def _createPublisher(self) -> Publisher:
        """
        The publisher to send data to.
        The format of the topic is the following: moteusmotor_<can_id>_from_can
        """
        topic_name = self.config.getCanTopicName()
        # Size of queue is 1. All additional ones are dropped
        publisher = self._ros_node.create_publisher(std_msgs.msg.String, topic_name, 1)

        return publisher

    def dataInCallback(self, msg: String) -> None:
        """
        Update the motor state. Mutex protected,
        meaning that no one can go into any other "critical section"
        of code that also has a mutex protecting it.
        """
        with self.mutex_lock:
            json_string = msg.data
            self.updateMotorState(json_string)

    def updateMotorState(self, raw_json_string: str) -> None:
        """
        Update the motor's variables from
        the input JSON string
        """

        json_helper = MoteusDataInJsonHelper()
        json_helper.buildHelper(raw_json_string)

        self.position = json_helper.getPosition()
        self.velocity = json_helper.getVelocity()
        self.feedforward_torque = json_helper.feedforward_torque
        self.kp_scale = json_helper.kp_scale
        self.kd_scale = json_helper.kd_scale
        self.max_torque = json_helper.max_torque
        self.watchdog_timeout = json_helper.watchdog_timeout
        self.accel_limit = json_helper.accel_limit
        self.fixed_voltage_override = json_helper.fixed_voltage_override
        self.set_stop = json_helper.set_stop

    def publishData(self, moteus_data: Any) -> None:
        """
        Publishes the data from the moteus controller
        """
        with self.mutex_lock:
            try:
                if not self._ros_node.context.ok():
                    return

                json_helper = MoteusDataOutJsonHelper()
                json_helper.can_id = self.config.can_id
                json_helper.position = moteus_data.values[moteus.Register.POSITION]
                json_helper.velocity = moteus_data.values[moteus.Register.VELOCITY]
                json_helper.torque = moteus_data.values[moteus.Register.TORQUE]
                json_helper.temperature = moteus_data.values[moteus.Register.TEMPERATURE]
                json_helper.power = moteus_data.values[moteus.Register.POWER]
                json_helper.input_voltage = moteus_data.values[moteus.Register.VOLTAGE]
                json_helper.q_current = moteus_data.values[moteus.Register.Q_CURRENT]
                json_helper.d_current = moteus_data.values[moteus.Register.D_CURRENT]

                # self._rosNode.get_logger().info(str(moteusData.values))

                # TODO: We need to update firmware to get POWER information (7/1/2024)
                # https://github.com/mjbots/moteus/releases
                # https://discord.com/channels/633996205759791104/722434939676786688/1252380387783610428
                # self._rosNode.get_logger().info(str(moteusData.values.keys()))

                json_string = json_helper.buildJSONString()

                msg = String()
                msg.data = json_string

                self._publisher.publish(msg)
            except Exception as error:  # pylint: disable=broad-exception-caught
                # This is used to handle any errors in order to prevent the thread from dying
                # Specifically, when we crtl-c we want the motors to be set_stop(), but if this
                # thread crashes we cannot do that. So we catch any errors
                self._ros_node.get_logger().info("Failed to publish motor data")
                self._ros_node.get_logger().info(str(error))
