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
      - Subscribes to /fix (GPS).
      - Ignores fixes for 15s after startup to allow stabilization.
      - Captures the first valid fix after 15s.
      - Publishes anchor lat/lon/alt on /anchor_position with transient local QoS.
      - Does not shut down after setting anchor. It remains alive unless forcibly stopped.
    """

    def __init__(self) -> None:
        super().__init__("gps_anchor_node")

        self.anchor_set = False
        self.anchor_lat = 0.0
        self.anchor_lon = 0.0
        self.anchor_alt = 0.0

        self.start_time = self.get_clock().now()
        self.gps_sub = self.create_subscription(NavSatFix, "/fix", self.gpsCallback, 10)

        transient_local_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.anchor_pub = self.create_publisher(
            Float64MultiArray, "/anchor_position", transient_local_qos
        )

        self.get_logger().info(
            "GpsAnchorNode initialized. Waiting for first valid GPS fix after 15s..."
        )

    def gpsCallback(self, msg: NavSatFix) -> None:
        if self.anchor_set:
            return

        time_passed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if time_passed < 15.0:
            return  # skip until 15s have elapsed

        # If fix is valid (>= 0 => at least a standard fix)
        if msg.status.status >= 0:
            self.anchor_lat = msg.latitude
            self.anchor_lon = msg.longitude
            self.anchor_alt = msg.altitude
            self.anchor_set = True

            self.get_logger().info(
                colorStr(
                    f"Anchor lat/lon set to: {self.anchor_lat:.6f}, {self.anchor_lon:.6f} "
                    f"(alt: {self.anchor_alt:.2f}) after {time_passed:.1f}s",
                    ColorCodes.GREEN_OK,
                )
            )

            # Publish anchor
            anchor_msg = Float64MultiArray()
            anchor_msg.data = [self.anchor_lat, self.anchor_lon, self.anchor_alt]
            self.anchor_pub.publish(anchor_msg)

            # Optionally stop subscribing if we only need the first fix
            self.destroy_subscription(self.gps_sub)
            self.get_logger().info(
                colorStr("Stopped GPS subscription after setting anchor.", ColorCodes.BLUE_OK)
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GpsAnchorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.get_logger().info(colorStr("Shutting down gps anchor node", ColorCodes.BLUE_OK))
    finally:
        # DO NOT call sys.exit(0)!
        # Let the node remain or gracefully exit with the rest of the launch.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
