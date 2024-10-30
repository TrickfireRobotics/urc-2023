#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class DecisionMakingNode(Node):
    def __init__(self):
        super().__init__("decision_making_node")

        # Publisher to send commands to Control Node
        self.control_pub = self.create_publisher(String, "/control_commands", 10)

        # Subscribers to listen for input from other nodes
        self.create_subscription(
            Bool, "/obstacle_detected", self.collision_handling, 10
        )
        self.create_subscription(Bool, "/target_lost", self.target_reacquisition, 10)

        # Initialize state variables
        self.obstacle_detected = False
        self.target_lost = False

    def collision_handling(self, msg):
        """Handles obstacle detection and decides on stopping or rerouting."""
        self.obstacle_detected = msg.data  # Boolean: True if obstacle detected
        if self.obstacle_detected:
            self.get_logger().info("Obstacle detected! Stopping rover.")
            self.control_pub.publish(String(data="STOP"))
        else:
            self.get_logger().info("Path clear, resuming navigation.")
            self.control_pub.publish(String(data="RESUME"))

    def target_reacquisition(self, msg):
        """Handles reacquiring target if lost and reorienting if needed."""
        self.target_lost = msg.data  # Boolean: True if target is lost
        if self.target_lost:
            self.get_logger().info("Target lost, initiating search.")
            self.control_pub.publish(String(data="SEARCH_FOR_TARGET"))
        else:
            self.get_logger().info("Target reacquired, resuming tracking.")
            self.control_pub.publish(String(data="RESUME_TRACKING"))


def main(args=None):
    rclpy.init(args=args)
    decision_making_node = DecisionMakingNode()

    try:
        rclpy.spin(decision_making_node)
    except KeyboardInterrupt:
        pass
    finally:
        decision_making_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
