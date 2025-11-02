import sys

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry  # todo, incorporate odometry to make position more accurate

# todo, subscribe to imu to get rotation information
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Float64MultiArray

from lib.color_codes import ColorCodes, colorStr


#'zed/zed_node/imu/data'
class Localization(Node):

    def __init__(self) -> None:
        super().__init__("localization_node")
        # the localization node keeps all pose information in float64 arrays
        # for fast internal math, but publishes in data types like posestamped, point, and quaternion for clarity

        #   POSITION DATA [x,y,z] or [lon,lat,alt]
        # first gps coords we recieve
        self._init_pos = Float64MultiArray()
        self._init_pos.data = [0, 0, 0]
        self._first_gps_recieved = False
        # local position is the cumulative change in our position from when the rover powered on
        self._local_pos = Float64MultiArray()
        self._local_pos.data = [0, 0, 0]
        # global position is where we are in reference to the enitre world
        self._global_pos = Float64MultiArray()
        self._global_pos.data = [0, 0, 0]

        #   ROTATION DATA [x,y,z,w]
        # local rotation is the cumulative change in rotation from when the rover powered on
        self._local_rot = Float64MultiArray()
        self._local_rot.data = [0, 0, 0, 0]
        # global rotation is determined using 2 gps readings, which lets us establish rotation in relation to the world
        self._global_rot = Float64MultiArray()
        self._global_rot.data = [0, 0, 0, 0]

        # internal variables declared, 'launching' refers to adding subscriptions and publishers
        self.get_logger().info(colorStr("Launching localization_node", ColorCodes.BLUE_OK))

        #   SUBSCRIPTIONS
        # supposedly 10 in the subscription refers to the queue size, but who knows what that means
        self.gps_sub = self.create_subscription(NavSatFix, "/anchor_position", self.processGPS, 10)
        self.gps_sub = self.create_subscription(Imu, "/zed/zed_node/imu/data", self.processIMU, 10)

        #   PUBLISHERS
        self.local_pos_pub = self.create_publisher(Vector3, "loc_local_position", 10)
        self.global_pos_pub = self.create_publisher(Vector3, "loc_global_position", 10)
        self.local_rot_pub = self.create_publisher(Quaternion, "loc_local_rotation", 10)
        self.global_rot_pub = self.create_publisher(Quaternion, "loc_global_rotation", 10)

        # local and global timestamped and bundled pose info for ros2 bags and mapping
        self.local_pose = self.create_publisher(PoseStamped, "loc_local_pose", 10)
        self.global_pose = self.create_publisher(PoseStamped, "loc_local_pose", 10)

        #   TIMERS
        # Timer to periodically publish the robot's pose for debug and mapping purposes
        self.timer = self.create_timer(0.1, self.publishPose)

    def processGPS(self, msg: Float64MultiArray) -> None:
        """
        Updates self.local_position and self.init_position from gps.
        """
        # recieve y,x,z from navsat fix ([self.anchor_lat, self.anchor_lon, self.anchor_alt])
        if not self._first_gps_recieved:
            self._init_pos.data = [msg.data[1], msg.data[0], msg.data[2]]
            self._first_gps_recieved = True
        else:
            self.updateLocalPos(lat=msg.data[0], lon=msg.data[1], alt=msg.data[2])

        self.updateGlobalPos(lat=msg.data[0], lon=msg.data[1], alt=msg.data[2])
        self.publishPosition()

    def processIMU(self, msg: Imu) -> None:
        content = "quaternion from imu" + msg.orientation
        self.get_logger().info(colorStr(content, ColorCodes.GREEN_OK))
        self.current_pose.pose.orientation = msg.orientation
        self.local_rot_pub.publish(msg.orientation)
        # could also use this info:
        # msg.linear_acceleration
        # msg.angular_velocity

    def publishLocalPosition(self) -> None:
        contents = self._local_pos
        msg = Vector3()  # Create the message
        msg.x = contents[1]  # x = lon
        msg.y = contents[0]  # y = lat
        msg.z = contents[2]  # z = alt
        self.local_pos_pub.publish(msg)

    def publishGlobalPosition(self) -> None:
        contents = self._global_pos
        msg = Vector3()  # Create the message
        msg.x = contents[1]  # x = lon
        msg.y = contents[0]  # y = lat
        msg.z = contents[2]  # z = alt
        self.global_pos_pub.publish(msg)

    def publishPosition(self) -> None:
        self.publishLocalPosition()
        self.publishGlobalPosition()

    def publishStampedPose(self) -> None:
        contents = self._local_pos
        # fill position information from current local position
        self.current_pose.pose.position.x = contents[1]  # x = lon
        self.current_pose.pose.position.y = contents[0]  # y = lat
        self.current_pose.pose.position.z = contents[2]  # z = alt
        # self.current_pose.pose.orientation is updated whenever we get new imu data above
        # self.current_pose.header.stamp update timestamp for publication
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        # self.current_pose.header.frame_id
        pass

    def updateLocalPos(self, lon: Float64, lat: Float64, alt: Float64) -> None:
        self._local_pos.data = [
            lon - self._init_pos.data[1],
            lat - self._init_pos.data[0],
            alt - self._init_pos.data[2],
        ]

    def updateGlobalPos(self, lon: Float64, lat: Float64, alt: Float64) -> None:
        self._global_pos.data = [
            lon,
            lat,
            alt,
        ]

    def updateOrientation(self, x: Float64, y: Float64, z: Float64, w: Float64) -> None:
        pass

    def updatePose(self) -> None:
        pass

    # def global_gps_loc_to_local_offset(self):


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        localization = Localization()
        rclpy.spin(localization)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        localization.get_logger().info(
            colorStr("Shutting down localization_node", ColorCodes.BLUE_OK)
        )
        localization.destroy_node()
        sys.exit(0)
