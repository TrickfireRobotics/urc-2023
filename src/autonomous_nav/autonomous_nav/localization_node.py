"""
Role: This script captures the first valid GPS fix (the "anchor" position)
but waits 15 seconds from startup before accepting any fix.
Once captured, it publishes the anchor lat/lon/alt on a transient local topic.

Dependencies:
    - rclpy: ROS 2 client library for Python.
    - sensor_msgs.msg: NavSatFix for reading GPS data.
    - std_msgs.msg: Float64MultiArray for publishing anchor data.
"""

import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from lib.color_codes import ColorCodes, colorStr


class GpsAnchorNode(Node):
    """
    A ROS 2 node for handling the initial GPS anchor.

    This node:
      - Subscribes to the /fix topic (GPS).
      - Ignores any fixes for 15s after startup to allow GPS to stabilize.
      - Captures the first valid fix after that 15s window.
      - Publishes the anchor lat/lon/alt on a transient local topic,
        ensuring late subscribers still receive the message.
    """

    def __init__(self) -> None:
        super().__init__("gps_anchor_node")

        # State to track whether we've set the anchor yet
        self.anchor_set = False
        self.anchor_lat = 0.0
        self.anchor_lon = 0.0
        self.anchor_alt = 0.0

        # Record the node's start time for the 15s warm-up
        self.start_time = self.get_clock().now()

        # Set up subscriber for GPS
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 10)

        # Create a transient local QoS profile so late subscribers get the last message
        transient_local_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publisher for the anchor data
        self.anchor_pub = self.create_publisher(
            Float64MultiArray, "/anchor_position", transient_local_qos
        )

        self.get_logger().info(
            "GpsAnchorNode initialized. Waiting for first valid GPS fix after 15s..."
        )

    def gpsCallback(self, msg: NavSatFix) -> None:
        # If we've already set an anchor, do nothing
        if self.anchor_set:
            return

        # Check how long it's been since startup
        time_passed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if time_passed < 15.0:
            # Not yet 15 seconds, skip any fix
            return

        # Now we are beyond 15s, check if there's a valid fix
        if msg.status.status >= 0:
            self.anchor_lat = msg.latitude
            self.anchor_lon = msg.longitude
            self.anchor_alt = msg.altitude
            self.anchor_set = True

            self.get_logger().info(
                f"Anchor lat/lon set to: {self.anchor_lat:.6f}, {self.anchor_lon:.6f} "
                f"(alt: {self.anchor_alt:.2f}) after {time_passed:.1f}s"
            )

            # Publish the anchor as a Float64MultiArray [lat, lon, alt]
            anchor_msg = Float64MultiArray()
            anchor_msg.data = [self.anchor_lat, self.anchor_lon, self.anchor_alt]
            self.anchor_pub.publish(anchor_msg)

            # Optionally stop subscribing if you only want the first valid fix
            self.destroy_subscription(self.gps_sub)
            self.get_logger().info("Stopped GPS subscription after setting anchor.")


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the GpsAnchorNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """
    rclpy.init(args=args)
    try:
        gps_anchor_node = GpsAnchorNode()
        rclpy.spin(gps_anchor_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        gps_anchor_node.get_logger().info(
            colorStr("Shutting down gps anchor node", ColorCodes.BLUE_OK)
        )
        gps_anchor_node.destroy_node()
        sys.exit(0)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
