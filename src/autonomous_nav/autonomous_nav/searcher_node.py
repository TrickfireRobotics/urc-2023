"""
This node activates when the navigation node is within 1 meter of the goal.
It spins the rover to look for an ArUco tag in a 5-meter radius around the goal position.
Once found, it publishes a status update.
"""

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Float32, String


class SearcherNode(Node):
    def __init__(self) -> None:
        super().__init__("searcher_node")

        self.activated = False
        self.goal = (0.0, 0.0)
        self.spin_timer = None
        self.found_timer = None

        # Publishers
        self.left_pub = self.create_publisher(Float32, "/left_wheel_velocity", 10)
        self.right_pub = self.create_publisher(Float32, "/right_wheel_velocity", 10)
        self.status_pub = self.create_publisher(String, "/searcher_status", 10)

        # Subscribers
        self.create_subscription(String, "/searcher_activation", self.activation_callback, 10)
        self.create_subscription(Pose2D, "/searcher_goal", self.goal_callback, 10)

    def activation_callback(self, msg: String) -> None:
        if msg.data == "activate" and not self.activated:
            self.activated = True
            self.get_logger().info("Searcher activated, starting to spin")
            self.start_spinning()
            # Simulate finding the tag after 10 seconds (placeholder for actual detection)
            self.found_timer = self.create_timer(10.0, self.found_callback)

    def goal_callback(self, msg: Pose2D) -> None:
        self.goal = (msg.x, msg.y)
        self.get_logger().info(f"Received goal position: {self.goal}")

    def start_spinning(self) -> None:
        # Publish velocities to spin the rover
        left_msg = Float32()
        left_msg.data = 1.0  # Adjust speed as needed
        right_msg = Float32()
        right_msg.data = -1.0
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        # Keep spinning with a timer
        self.spin_timer = self.create_timer(0.1, self.spin_callback)

    def spin_callback(self) -> None:
        if self.activated:
            left_msg = Float32()
            left_msg.data = 1.0
            right_msg = Float32()
            right_msg.data = -1.0
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
        else:
            self.stop_spinning()

    def stop_spinning(self) -> None:
        left_msg = Float32()
        left_msg.data = 0.0
        right_msg = Float32()
        right_msg.data = 0.0
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        if self.spin_timer:
            self.spin_timer.cancel()
            self.spin_timer = None

    def found_callback(self) -> None:
        self.get_logger().info("Tag found (simulated), publishing status")
        self.status_pub.publish(String(data="found"))
        self.activated = False
        self.stop_spinning()
        if self.found_timer:
            self.found_timer.cancel()
            self.found_timer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SearcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
