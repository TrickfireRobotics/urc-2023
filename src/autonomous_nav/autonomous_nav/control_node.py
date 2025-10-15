"""
This node is the middleman between the decision_making_node and the drivebase.
Subscribes to wheel velocity commands from decision_making_node and republishes to drivebase.
"""

from rclpy.node import Node
from std_msgs.msg import Float32


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__("control_node")

        # Velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Subscribers
        self.create_subscription(Float32, "/left_wheel_velocity", self.left_wheel_callback, 10)
        self.create_subscription(Float32, "/right_wheel_velocity", self.right_wheel_callback, 10)

        # Publishers
        self.left_wheel_pub = self.create_publisher(
            Float32, "/move_left_drivebase_side_message", 10
        )
        self.right_wheel_pub = self.create_publisher(
            Float32, "/move_right_drivebase_side_message", 10
        )

    def left_wheel_callback(self, msg: Float32) -> None:
        self.left_wheel_velocity = msg.data
        self.publish_left_wheel_commands()

    def right_wheel_callback(self, msg: Float32) -> None:
        self.right_wheel_velocity = msg.data
        self.publish_right_wheel_commands()

    def publish_right_wheel_commands(self) -> None:
        right_msg = Float32()
        right_msg.data = self.right_wheel_velocity
        self.right_wheel_pub.publish(right_msg)

    def publish_left_wheel_commands(self) -> None:
        left_msg = Float32()
        left_msg.data = self.left_wheel_velocity
        self.left_wheel_pub.publish(left_msg)
