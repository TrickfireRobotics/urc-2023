import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from lib.color_codes import ColorCodes
from lib.interface.robot_info import RobotInfo

from . import info_to_json_helper


class MissionControlUpdater(Node):

    def __init__(self) -> None:
        super().__init__("mission_control_updater_node")
        self.get_logger().info(
            ColorCodes.BLUE_OK + "Launching mission_control_updater_node" + ColorCodes.ENDC
        )

        self.publisher_to_mc = self.create_publisher(String, "mission_control_updater", 1)

        self.timer = self.create_timer(0.01, self.sendData)
        self.robot_info = RobotInfo(self)

    def sendData(self) -> None:
        set_of_can_id = {20, 21, 22, 23, 24, 25, 1, 2, 3, 4, 5}

        json_builder = info_to_json_helper.InfoToJSONHelper()

        for can_id in set_of_can_id:
            json_builder.addMoteusEntry(self.robot_info.getMotorState(can_id))

        msg = String()
        msg.data = json_builder.buildJSONString()
        self.publisher_to_mc.publish(msg)


def main(args: list[str] | None = None) -> None:
    """
    The entry point of the node.
    """

    rclpy.init(args=args)
    try:
        node = MissionControlUpdater()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # This is done when we ctrl-c the progam to shut it down
        node.get_logger().info(
            ColorCodes.BLUE_OK + "Shutting down mission_control_updater_node" + ColorCodes.ENDC
        )
        node.destroy_node()
        sys.exit(0)


if __name__ == "__main__":
    main()
