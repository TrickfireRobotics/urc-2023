"""
Role: Implements mission logic for science tasks.
      - Subscribes to hardware data (camera, sensors).
      - Publishes mission status, analysis results.
      - Calls sample collection services.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty  # or your custom service

from lib.color_codes import ColorCodes, colorStr

# from some_custom_msgs.srv import CollectSample
# from some_custom_msgs.msg import AnalysisResult


class ScienceMissionNode(Node):
    """
    A ROS 2 node for high-level science mission orchestration.
    """

    def __init__(self) -> None:
        super().__init__("science_mission_node")

        # Declare/read parameters
        self.declare_parameter("mission_status_topic", "/science/mission_status")
        self.declare_parameter("analysis_result_topic", "/science/analysis_result")
        self.declare_parameter("site_selection_topic", "/science/site_selection")
        # Add more if needed

        mission_status_topic = (
            self.get_parameter("mission_status_topic").get_parameter_value().string_value
        )
        analysis_result_topic = (
            self.get_parameter("analysis_result_topic").get_parameter_value().string_value
        )
        site_selection_topic = (
            self.get_parameter("site_selection_topic").get_parameter_value().string_value
        )

        # Subscriptions (camera feeds, GNSS, sensors, etc.)
        self.create_subscription(Image, "/science/camera_wide/image_raw", self.wideCamCallback, 10)
        self.create_subscription(
            Image, "/science/camera_closeup/image_raw", self.closeupCamCallback, 10
        )
        self.create_subscription(NavSatFix, "/science/gnss", self.gnssCallback, 10)
        self.create_subscription(Float32, "/science/soil_moisture", self.soilMoistureCallback, 10)
        self.create_subscription(Float32, "/science/subsurface_temp", self.tempCallback, 10)

        # Publishers
        self.mission_status_pub = self.create_publisher(String, mission_status_topic, 10)
        self.analysis_result_pub = self.create_publisher(String, analysis_result_topic, 10)
        self.site_selection_pub = self.create_publisher(String, site_selection_topic, 10)

        # Client to call sample collection service
        self.sample_collection_cli = self.create_client(Empty, "/science/collect_sample")
        # or self.create_client(CollectSample, '/science/collect_sample') if custom

        # Timer or main loop for mission logic
        self.timer = self.create_timer(1.0, self.controlLoop)

        self.get_logger().info("ScienceMissionNode initialized.")

    def controlLoop(self) -> None:
        """
        Periodic control loop for the mission logic.
        Example: check state, publish status, or request sample collection.
        """
        # Example placeholder:
        msg = String()
        msg.data = "Mission running. (Placeholder)"
        self.mission_status_pub.publish(msg)

    def wideCamCallback(self, msg: Image) -> None:
        """
        Callback for wide-angle camera data.
        """
        # TODO: Possibly process images, detect interesting features, etc.
        pass

    def closeupCamCallback(self, msg: Image) -> None:
        """
        Callback for close-up camera data.
        """
        # TODO: Possibly used for confirming sample collection or analyzing texture.
        pass

    def gnssCallback(self, msg: NavSatFix) -> None:
        """
        Callback for GNSS data (latitude, longitude, altitude).
        """
        # TODO: Store or use to decide site location, log it, etc.
        pass

    def soilMoistureCallback(self, msg: Float32) -> None:
        """
        Callback for soil moisture sensor data.
        """
        # TODO: Evaluate if threshold is met to proceed with sampling, etc.
        pass

    def tempCallback(self, msg: Float32) -> None:
        """
        Callback for subsurface temperature data.
        """
        # TODO: Possibly used in analysis to check for habitable conditions, etc.
        pass

    def requestSampleCollection(self) -> None:
        """
        Example function for calling the sample collection service.
        """
        if not self.sample_collection_cli.service_is_ready():
            self.get_logger().warn("Sample collection service not ready yet.")
            return

        request = Empty.Request()  # or your custom request
        future = self.sample_collection_cli.call_async(request)
        future.add_done_callback(self.sampleCollectionResponseCallback)

    def sampleCollectionResponseCallback(self, future: Future) -> None:
        """
        Handle the response from the sample collection service.
        """
        try:
            response = future.result()
            self.get_logger().info("Sample collection successful.")
        except Exception as e:
            self.get_logger().error(f"Sample collection service call failed: {e}")


def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize the rclpy context and run the ScienceMissionNode.

    Args:
        args (Optional[Any]): Command-line arguments passed to rclpy.init().
    """

    rclpy.init(args=args)
    try:
        science_mission_node = ScienceMissionNode()
        rclpy.spin(science_mission_node)  # prints callbacks
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        science_mission_node.get_logger().info(
            colorStr("Shutting down Science Mission Node", ColorCodes.BLUE_OK)
        )
        science_mission_node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
