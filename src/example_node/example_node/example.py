import sys


import rclpy  # Import the package
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

from lib.color_codes import ColorCodes, colorStr  # Import yummy colors
from lib.interface.robot_info import RobotInfo  # Read data
from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface  # Send data


class ExampleNode(Node):

    def __init__(self) -> None:
        # Call the parent's constructor and pass in the node's name
        super().__init__("my_example_node")

        # Print to the console with the color blue. Regular print() does not work
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.BLUE_OK))

        self.robotInfo = RobotInfo(self)
        self.robotInterface = RobotInterface(self)

        self.drivebaseSpeedInRadians = 6.28  # 6.28 rad/sec = 1.0 rev/sec

        # --- ROS TIMER EXAMPLE ---
        # Half a second delay between the calls to the method timer_callback()
        self.create_timer(0.5, self.timer_callback)

        # --- SUBSCRIBER EXAMPLE ---
        self.example_sub = self.create_subscription(
            String, "example_topic", self.example_sub_callback, 10
        )

        # --- PUBLISHER EXAMPLE ---
        self.example_pub = self.create_publisher(String, "example_topic", 10)
        self.example_pub_timer = self.create_timer(1.0, self.publishData)

    def timer_callback(self) -> None:
        # Tell the motor to move at a speed. We skip having to create a run config
        # Take a look at the runMotor() to see the config
        # In this case, runMotorSpeed() takes speed as in input in radians per second
        self.robotInterface.runMotorSpeed(
            MotorConfigs.FRONT_LEFT_DRIVE_MOTOR, self.drivebaseSpeedInRadians
        )

        # Get the motor's position
        motorPosition = self.robotInfo.getMotorState(MotorConfigs.FRONT_LEFT_DRIVE_MOTOR).position

        self.get_logger().info(
            "Example timer callback called with motor position " + str(motorPosition)
        )

    def example_sub_callback(self, msg: String) -> None:
        contents = msg.data
        self.get_logger().info("Example Subscriber Callback got the data: " + str(contents))

    def publishData(self) -> None:
        contents = "Hello this is data from example_topic"
        msg = String()  # Create the message
        msg.data = contents  # Fill in the payload
        self.example_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        # Create a new instance of our class
        example = ExampleNode()

        # Have ROS spin this node, meaning that this object will continuously
        # be part of the ROS event loop and such.
        rclpy.spin(example)
    except KeyboardInterrupt:  # Detects ctrl-c
        pass
    except ExternalShutdownException:
        # If needed, we could call methods from the instance we made
        # in order to shut somethings down before destroying the node
        example.get_logger().info(colorStr("Shutting down example_node node", ColorCodes.BLUE_OK))
        example.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
