import rclpy
from rclpy.node import Node

from robot_interface import RobotInterface

from std_msgs.msg import String


class Drivebase(Node):

    global botInterface

    def __init__(self):
        super().__init__('drivebase')

        global botInterface
        botInterface = RobotInterface(self)

        self.left_subscription = self.create_subscription(
            String, 'move_left_drivebase_side_message', self.move_left_side, 10)
        self.right_subscription = self.create_subscription(
            String, 'move_right_drivebase_side_message', self.move_right_side, 10)

    def move_left_side(self, msg):
        botInterface.leftFrontWheel(msg)
        botInterface.leftMiddleWheel(msg)
        botInterface.leftBackWheel(msg)

    def move_right_side(self, msg):
        botInterface.rightFrontWheel(msg)
        botInterface.rightMiddleWheel(msg)
        botInterface.rightBackWheel(msg)


def main(args=None):
    rclpy.init(args=args)

    drivebase = Drivebase()

    print("drivebase main")

    rclpy.spin(drivebase) # prints callbacks

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drivebase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
