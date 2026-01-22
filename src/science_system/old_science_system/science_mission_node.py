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

        # Mission state tracking
        self.mission_state = "IDLE"

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

        # Service to manually pause mission from control interface
        self.create_service(Empty, "/science/pause_mission", self.handlePauseMission)

        # Timer or main loop for mission logic
        self.timer = self.create_timer(1.0, self.controlLoop)

        self.get_logger().info("ScienceMissionNode initialized.")

    def controlLoop(self) -> None:
        """
        Periodic control loop for the mission logic.
        Example: check state, publish status, or request sample collection.
        """
        self.get_logger().info(f"Current mission state: {self.mission_state}")
        msg = String()
        msg.data = f"Mission state: {self.mission_state}"
        self.mission_status_pub.publish(msg)

    def setMissionState(self, state: str) -> None:
        """Updates the mission state and logs the change."""
        self.mission_state = state
        self.get_logger().info(f"Mission state updated to: {state}")

    def logMissionData(self, event: str) -> None:
        """Logs mission data and events."""
        self.get_logger().info(f"[MISSION LOG] {event}")

    def wideCamCallback(self, msg: Image) -> None:
        """
        Callback for wide-angle camera data.
        """
        self.logMissionData("Received wide-angle camera image.")
        # TODO: Possibly process images, detect interesting features, etc.
        pass

    def closeupCamCallback(self, msg: Image) -> None:
        """
        Callback for close-up camera data.
        """
        self.logMissionData("Received close-up camera image.")
        # TODO: Possibly used for confirming sample collection or analyzing texture.
        pass

    def gnssCallback(self, msg: NavSatFix) -> None:
        """
        Callback for GNSS data (latitude, longitude, altitude).
        """
        self.logMissionData(f"GNSS: lat={msg.latitude}, lon={msg.longitude}")
        # TODO: Store or use to decide site location, log it, etc.
        pass

    def soilMoistureCallback(self, msg: Float32) -> None:
        """
        Callback for soil moisture sensor data.
        """
        self.logMissionData(f"Soil moisture: {msg.data}")
        if msg.data > 20.0:
            self.get_logger().info("High moisture detected, initiating sample collection.")
            self.requestSampleCollection()
        # TODO: Evaluate if threshold is met to proceed with sampling, etc.
        pass

    def tempCallback(self, msg: Float32) -> None:
        """
        Callback for subsurface temperature data.
        """
        self.logMissionData(f"Subsurface temperature: {msg.data}")
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
            if response is None:
                self.get_logger().error("Sample collection service returned no response.")
                return
            self.get_logger().info("Sample collection successful.")
            self.setMissionState("ANALYZING")
        except Exception as e:
            self.get_logger().error(f"Sample collection service call failed: {e}")
            self.setMissionState("ERROR")

    def handlePauseMission(
        self, request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        """
        Pauses the mission when called via service.
        """
        self.get_logger().warn("Mission manually paused.")
        self.setMissionState("PAUSED")
        return response


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
