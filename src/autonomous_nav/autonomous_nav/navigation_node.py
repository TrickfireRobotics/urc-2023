import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # Declare and retrieve waypoints parameter
        self.declare_parameter("waypoints", [])
        self.waypoints = (
            self.get_parameter("waypoints").get_parameter_value().double_array_value
        )
        self.current_position = None

        # Publisher for sending goal positions
        self.goal_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def update_position(self, new_position):
        # Update current position and plan path to the next waypoint
        self.current_position = new_position
        self.plan_path()

    def plan_path(self):
        if self.current_position:
            next_goal = self.get_next_waypoint()
            if next_goal:
                goal_msg = PoseStamped()
                # Set goal pose based on next waypoint
                goal_msg.pose.position.x = next_goal[0]
                goal_msg.pose.position.y = next_goal[1]
                self.goal_publisher.publish(goal_msg)
                self.get_logger().info(f"Published goal: {next_goal}")

    def get_next_waypoint(self):
        # Retrieve the next waypoint; simple example uses first in list
        return self.waypoints.pop(0) if self.waypoints else None


def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
