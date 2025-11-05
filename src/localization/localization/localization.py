import sys

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry  # todo, incorporate odometry to make position more accurate

# todo, subscribe to imu to get rotation information
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Float64MultiArray

from lib.color_codes import ColorCodes, colorStr


class Coords:
    def __init__(self, x: Float64 = 0, y: Float64 = 0, z: Float64 = 0) -> None:
        self._array = Float64MultiArray()
        self._array.data = [x, y, z]

    def setX(self, x: Float64) -> None:
        self._array[0] = x

    def getX(self) -> Float64:
        return self._array[0]

    def setLon(self, lon: Float64) -> None:
        self._array[0] = lon

    def getLon(self) -> Float64:
        return self._array[0]

    def setY(self, y: Float64) -> None:
        self._array[1] = y

    def getY(self) -> Float64:
        return self._array[1]

    def setLat(self, lat: Float64) -> None:
        self._array[1] = lat

    def getLat(self) -> Float64:
        return self._array[1]

    def setZ(self, z: Float64) -> None:
        self._array[2] = z

    def getZ(self) -> Float64:
        return self._array[2]

    def setAlt(self, alt: Float64) -> None:
        self._array[2] = alt

    def getAlt(self) -> Float64:
        return self._array[2]

    def setCoords(self, lon: Float64, lat: Float64, alt: Float64) -> None:
        self._array[0] = lon
        self._array[1] = lat
        self._array[2] = alt

    def getCoords(self) -> Float64MultiArray:
        return self._array

    def setData(self, x: Float64, y: Float64, z: Float64) -> None:
        self._array[0] = x
        self._array[1] = y
        self._array[2] = z

    def getData(self) -> Float64MultiArray:
        return self._array

    def toVector3(self) -> Vector3:
        out = Vector3()
        out.x = self.getX()
        out.y = self.getY()
        out.z = self.getZ()
        return out

    def toPoint(self) -> Point:
        out = Point()
        out.x = self.getX()
        out.y = self.getY()
        out.z = self.getZ()
        return out


class Orientation:
    def __init__(self, x: Float64 = 0, y: Float64 = 0, z: Float64 = 0, w: Float64 = 0) -> None:
        self._array = Float64MultiArray()
        self._array.data = [x, y, z, w]

    def setX(self, x: Float64) -> None:
        self._array[0] = x

    def getX(self) -> Float64:
        return self._array[0]

    def setY(self, y: Float64) -> None:
        self._array[1] = y

    def getY(self) -> Float64:
        return self._array[1]

    def setZ(self, z: Float64) -> None:
        self._array[2] = z

    def getZ(self) -> Float64:
        return self._array[2]

    def setW(self, w: Float64) -> None:
        self._array[3] = w

    def getW(self) -> Float64:
        return self._array[3]

    def setData(self, x: Float64, y: Float64, z: Float64, w: Float64) -> None:
        self._array[0] = x
        self._array[1] = y
        self._array[2] = z
        self._array[3] = w

    def setDataFromQuat(self, data: Quaternion) -> None:
        self._array[0] = data.x
        self._array[1] = data.y
        self._array[2] = data.z
        self._array[3] = data.w

    def getData(self) -> Float64MultiArray:
        return self._array

    def toQuaternion(self) -> Quaternion:
        out = Quaternion()
        out.x = self.getX()
        out.y = self.getY()
        out.z = self.getZ()
        out.w = self.getW()
        return out


#'zed/zed_node/imu/data'
class Localization(Node):

    def __init__(self) -> None:
        super().__init__("localization_node")
        # the localization node keeps all pose information in float64 arrays
        # for fast internal math, but publishes in data types like posestamped, point, and quaternion for clarity

        #   POSITION DATA [x,y,z] or [lon,lat,alt]
        # first gps coords we recieve
        self._init_pos = Coords()
        self._first_gps_recieved = False
        # local position is the cumulative change in our position from when the rover powered on
        self._local_pos = Coords()
        # global position is where we are in reference to the enitre world
        self._global_pos = Coords()

        #   ROTATION DATA [x,y,z,w]
        # local rotation is the cumulative change in rotation from when the rover powered on
        self._local_rot = Orientation()
        # global rotation is determined using 2 gps readings, which lets us establish rotation in relation to the world
        self._global_rot = Orientation()

        # internal variables declared, 'launching' refers to adding subscriptions and publishers
        self.get_logger().info(colorStr("Launching localization_node", ColorCodes.BLUE_OK))

        #   SUBSCRIPTIONS
        # supposedly 10 in the subscription refers to the queue size, but who knows what that means
        self.gps_sub = self.create_subscription(NavSatFix, "/anchor_position", self.processGPS, 10)
        self.gps_sub = self.create_subscription(Imu, "/zed/zed_node/imu/data", self.processIMU, 10)

        #   PUBLISHERS
        self.local_pos_pub = self.create_publisher(Vector3, "loc_local_position", 10)  # done
        self.global_pos_pub = self.create_publisher(Vector3, "loc_global_position", 10)  # done
        self.local_rot_pub = self.create_publisher(Quaternion, "loc_local_rotation", 10)  # done
        self.global_rot_pub = self.create_publisher(Quaternion, "loc_global_rotation", 10)

        # local and global timestamped and bundled pose info for ros2 bags and mapping
        self.local_pose_pub = self.create_publisher(PoseStamped, "loc_local_pose", 10)
        self.global_pose_pub = self.create_publisher(PoseStamped, "loc_global_pose", 10)

        #   TIMERS
        # Timer to periodically publish the robot's pose for debug and mapping purposes
        self.timer = self.create_timer(0.1, self.publishLocalPose)

    def processGPS(self, msg: Float64MultiArray) -> None:
        """
        Updates self.local_position and self.init_position from gps.
        """
        # recieve y,x,z from navsat fix ([self.anchor_lat, self.anchor_lon, self.anchor_alt])
        if not self._first_gps_recieved:
            self._init_pos.setCoords(lat=msg.data[0], lon=msg.data[1], alt=msg.data[2])
            self._first_gps_recieved = True
        else:
            self.updateLocalPos(lat=msg.data[0], lon=msg.data[1], alt=msg.data[2])

        self._global_pos.setCoords(lat=msg.data[0], lon=msg.data[1], alt=msg.data[2])
        self.publishPosition()

    def processIMU(self, msg: Imu) -> None:
        content = "quaternion from imu" + msg.orientation
        self.get_logger().info(colorStr(content, ColorCodes.GREEN_OK))
        self._local_rot.setDataFromQuat(msg.orientation)
        self.local_rot_pub.publish(msg.orientation)
        # could also use this info:
        # msg.linear_acceleration
        # msg.angular_velocity

    def publishPosition(self) -> None:
        self.local_pos_pub.publish(self._local_pos.toVector3())
        self.local_pos_pub.publish(self._global_pos.toVector3())

    def publishLocalPose(self) -> None:
        time = self.get_clock().now().to_msg()
        frameid = "map"
        msg = self.getPoseStamped(self._local_pos, self._local_rot, time, frameid)
        self.local_pose_pub.publish(msg)

    def getPoseStamped(
        self, pos: Coords, rot: Orientation, time: Time, frameid: str
    ) -> PoseStamped:
        out = PoseStamped()
        out.pose.position = pos.toPoint()
        out.pose.orientation = rot.toQuaternion()
        out.header.stamp = time
        out.header.frame_id = frameid
        return out

    def updateLocalPos(self, lon: Float64, lat: Float64, alt: Float64) -> None:
        self._local_pos.setCoords(
            lon - self._init_pos.getLon(),
            lat - self._init_pos.getLat(),
            alt - self._init_pos.getAlt(),
        )

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
