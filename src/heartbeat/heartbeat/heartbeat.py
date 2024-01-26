import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class Heartbeat(Node):
    def __init__(self):
        # initialize heartbeat node
        super().__init__("heartbeat_node")
        self.get_logger().info("Hi from heartbeat!")

        # flag to store connection status
        self.connection_lost = False

        # subscriber for the dummy node status topic
        self.subscription = self.create_subscription(
            String, # message type
            '/heartbeat', # topic name
            self.heartbeat_callback, # callback function
            10  # Quality of Service (QoS) profile
        )

        # check connection every 1 second
        self.timer = self.create_timer(1.0, self.check_connection) 
# assume that there's no connection until there is
    def check_connection(self):
        if self.connection_lost:
            self.get_logger().warning("Connection lost")
        else:
            self.get_logger().info("Connection active")

    def heartbeat_callback(self, msg):
        # update the flag based on the message received
        if msg.data == '1':
            self.get_logger().info("Connection active")
        else:
            self.get_logger().warning("Connection lost")

def main(args=None):
    rclpy.init(args=args)
    heartbeat_node = Heartbeat()
    rclpy.spin(heartbeat_node)
    heartbeat_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    Heartbeat.main()
