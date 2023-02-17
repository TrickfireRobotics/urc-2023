import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Drivebase(Node):

    def __init__(self):
        super().__init__('drivebase')
        self.subscription = self.create_subscription(
            String, 'controller', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    drivebase = Drivebase()

    print("drivebase test")
    print(drivebase.get_subscriptions_info_by_topic('controller'))

    rclpy.spin_once(drivebase) # prints callbacks


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drivebase.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
