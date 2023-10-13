import math
from typing import get_args, Literal, TYPE_CHECKING

from rclpy.node import Node
from std_msgs.msg import Float32


# Because I don't want to write a thousand getters, the getters are made
# dynamically. But because of this, you really have to shoehorn things to make
# pylance (autofill) detect something as a valid attribute. Unfortunately,
# this workaround won't be able to add types to attributes.

# Anything in this literal will be picked up as a valid attribute.
ATTRIBUTES = Literal[
    "front_left_drive_motor_velocity",
    "mid_left_drive_motor_velocity",
    "back_left_drive_motor_velocity",
    "front_right_drive_motor_velocity",
    "mid_right_drive_motor_velocity",
    "back_right_drive_motor_velocity",
    "front_left_drive_motor_torque",
    "mid_left_drive_motor_torque",
    "back_left_drive_motor_torque",
    "front_right_drive_motor_torque",
    "mid_right_drive_motor_torque",
    "back_right_drive_motor_torque",
    "front_left_drive_motor_temperature",
    "mid_left_drive_motor_temperature",
    "back_left_drive_motor_temperature",
    "front_right_drive_motor_temperature",
    "mid_right_drive_motor_temperature",
    "back_right_drive_motor_temperature"
]

# Every type in this tuple is the type used for the topic at the same index in
# ATTRIBUTES
TYPES = (
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32,
    Float32
)


def rotations_to_radians(rotations: float):
    """Converts the given rotations to radians.

    Args:
        rotations (float): The rotations to convert

    Returns:
        float: The given rotations converted to radians.
    """
    return rotations * 2 * math.pi


# Every method in this tuple is the method used to convert the data in the
# message used for the topic at the same index in ATTRIBUTES. None signifies no
# converter is needed.
CONVERTERS = (
    rotations_to_radians,
    rotations_to_radians,
    rotations_to_radians,
    rotations_to_radians,
    rotations_to_radians,
    rotations_to_radians,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None,
    None
)


class RobotInfo:
    """RobotInfo is a class that subscribes to relevant topics and provides an
    attribute for each value.

    Any attribute that hasn't been assigned a value by an event yet will have a
    value of `None`.
    """
    def __init__(self, node: Node):
        """Creates a RobotInfo instance.

        Args:
            node (Node): The node RobotInfo uses to subscribe.
        """
        # Create subscriptions
        zipped = zip(get_args(ATTRIBUTES), TYPES, CONVERTERS)
        for topic, type, converter in zipped:
            callback = self._create_callback(converter)
            node.create_subscription(type, topic, callback, 10)

            # Initialize attribute
            setattr(self, topic, None)

    # TYPE_CHECKING is true while coding (aka while type checking) and false
    # during runtime, this ensures things that don't need to be defined during
    # runtime aren't defined.
    if TYPE_CHECKING:
        # This method's sole purpose is to tell autofill that a certain
        # attribute exists. Anything in the literal on name will be picked up
        # as an attribute.
        def __getattr__(self, _: ATTRIBUTES):
            pass

    def _create_callback(self, converter):
        def wrapper(msg):
            # setattr(self, "name", "example value") is equivalent to
            # self.name = "example value"
            # so it actually sets the attribute on the RobotInfo object itself.
            data = msg.data
            if converter is not None:
                data = converter(data)
            setattr(self, msg.get_topic_name(), data)

        return wrapper
