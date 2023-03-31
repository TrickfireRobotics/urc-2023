import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class LatencyPub(Node):

    def __init__(self):
        super().__init__('latency_pub')
        self.publisher_ = self.create_publisher(String, 'latency', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %s' % self.get_clock().now()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    latency_pub = LatencyPub()

    rclpy.spin(latency_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    latency_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()