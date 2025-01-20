"""
Role: Aggregates all science-related hardware drivers in one node.
      - Publishes sensor data (camera, GNSS, moisture, etc.)
      - Exposes services/actions for sample collection mechanism.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32
from std_srvs.srv import Empty  # or define your own sample collection service

from lib.color_codes import ColorCodes, colorStr

# from some_custom_msgs.srv import CollectSample  # Example if you define your own
# from rclpy.action import ActionServer
# from some_custom_msgs.action import CollectSampleAction


class ScienceHardwareNode(Node):
    """
    A ROS 2 node for interfacing with the rover's science hardware.
    """

    def __init__(self) -> None:
        super().__init__("science_hardware_node")

        # Declare/read parameters (camera topics, sensor topics, etc.)
        self.declare_parameter("camera_wide_topic", "/science/camera_wide/image_raw")
        self.declare_parameter("camera_closeup_topic", "/science/camera_closeup/image_raw")
        self.declare_parameter("gnss_topic", "/science/gnss")
        self.declare_parameter("soil_moisture_topic", "/science/soil_moisture")
        self.declare_parameter("subsurface_temp_topic", "/science/subsurface_temp")
        self.declare_parameter("sample_collection_service", "/science/collect_sample")

        camera_wide_topic = (
            self.get_parameter("camera_wide_topic").get_parameter_value().string_value
        )
        camera_closeup_topic = (
            self.get_parameter("camera_closeup_topic").get_parameter_value().string_value
        )
        gnss_topic = self.get_parameter("gnss_topic").get_parameter_value().string_value
        soil_moisture_topic = (
            self.get_parameter("soil_moisture_topic").get_parameter_value().string_value
        )
        subsurface_temp_topic = (
            self.get_parameter("subsurface_temp_topic").get_parameter_value().string_value
        )
        sample_collection_srv_name = (
            self.get_parameter("sample_collection_service").get_parameter_value().string_value
        )

        # Publishers for hardware data
        self.camera_wide_pub = self.create_publisher(Image, camera_wide_topic, 10)
        self.camera_closeup_pub = self.create_publisher(Image, camera_closeup_topic, 10)
        self.gnss_pub = self.create_publisher(NavSatFix, gnss_topic, 10)
        self.moisture_pub = self.create_publisher(Float32, soil_moisture_topic, 10)
        self.temp_pub = self.create_publisher(Float32, subsurface_temp_topic, 10)

        # Example service for sample collection
        # If you have a custom service or action, replace with appropriate definitions
        self.sample_collection_srv = self.create_service(
            Empty,  # or CollectSample if custom
            sample_collection_srv_name,
            self.handleSampleCollection,
        )

        # Timers or other internal loops to read hardware and publish
        self.publish_timer = self.create_timer(0.1, self.publishHardwareData)

        self.get_logger().info("ScienceHardwareNode initialized.")

    def publishHardwareData(self) -> None:
        """Periodically publishes data from sensors."""
        # TODO: Actually read from hardware or bridging library

        # Dummy example: publish a blank NavSatFix
        gnss_msg = NavSatFix()
        # fill with real sensor data
        self.gnss_pub.publish(gnss_msg)

        # Similarly for soil moisture or images
        # image_msg = Image()
        # self.camera_wide_pub.publish(image_msg)
        # etc.

    def handleSampleCollection(self, request, response) -> Empty.Response:
        """Handles the sample collection service call."""
        self.get_logger().info("Received request to collect a sample.")
        # TODO: Trigger real hardware (e.g. auger, scoop) to collect sample
        # Return success/failure in the service response if needed
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = ScienceHardwareNode()
        rclpy.spin(node)  # prints callbacks
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.get_logger().info(colorStr("Shutting down Science Hardware Node", ColorCodes.BLUE_OK))
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
