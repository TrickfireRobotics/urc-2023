import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DummyNode(Node):
    def __init__(self):
        # initialize dummy node
        super().__init__("dummy_node")
        self.get_logger().info("Hi from dummy node!")

        # simulate connection loss after 5 seconds
        self.create_timer(5.0, self.simulate_connection_loss)

        # publisher to the dummy_node_status
        self.publisher = self.create_publisher(Bool, 'dummy_node_status', 10)

    def simulate_connection_loss(self):
        self.get_logger().warning("Simulating connection loss")
        msg = Bool()
        # set the flag to true to indicate connection loss
        msg.data = True 
        # publish the message to the topic
        self.publisher.publish(msg) 

def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyNode()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()