import rclpy
from rclpy.node import Node

from ...interface import botInterface

from std_msgs.msg import String


class Drivebase(Node):

    botInterface: botInterface = botInterface(self)

    def __init__(self):
        super().__init__('drivebase')
        self.left_subscription = self.create_subscription(
            String, 'controller_left', self.move_left_side, 10)
        self.right_subscription = self.create_subscription(
            String, 'controller_right', self.move_right_side, 10)
        self.subscription  # prevent unused variable warning

    def move_left_side(self, msg):
        botInterface.leftFrontWheelForward(msg)
        botInterface.leftMiddleWheelForward(msg)
        botInterface.leftBackWheelForward(msg)
    
    def move_right_side(self, msg):
        botInterface.rightFrontWheelForward()
        botInterface.rightMiddleWheelForward()
        botInterface.rightBackWheelForward()


def main(args=None):
    rclpy.init(args=args)

    drivebase = Drivebase()

    print("drivebase test")

    rclpy.spin(drivebase) # prints callbacks


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drivebase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
