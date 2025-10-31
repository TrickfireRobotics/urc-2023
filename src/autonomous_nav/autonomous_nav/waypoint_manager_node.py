import sys, csv, math


import rclpy  # Import the package
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from rclpy.qos import QosProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from lib.color_codes import ColorCodes, colorStr  # Import yummy colors
from lib.interface.robot_info import RobotInfo  # Read data
from lib.configs import MotorConfigs
from lib.interface.robot_interface import RobotInterface  # Send data



def wgs84_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Tiny flat-earth geometry conversion"""
    R = 6378137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians((lat + lat0)/2))
    y = math.radians(lat - lat0) * R
    return x, y

class WaypointManagerNode(Node):

    def __init__(self) -> None:
        super().__init__("waypoint_manager_node")
        self.get_logger().info(colorStr("Launching example_node node", ColorCodes.RED_OK))

        self.robotInfo = RobotInfo(self)
        self.robotInterface = RobotInterface(self)
        
        self.declare_parameter("waypoint_file", "waypoints.csv")
        
        # temporary hardcoded origin
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)

        csv_path = self.get_parameter("waypoint_file").value
        lat0 = self.get_parameter('origin_lat').value
        lon0 = self.get_parameter('origin_lon').value
        # --- SUBSCRIBER EXAMPLE ---
        self.example_sub = self.create_subscription(
            String, "example_topic", self.example_sub_callback, 10
        )

        # custom quality of service, transient local to make sure latched messages are received
        qos = QosProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
                )
        # --- PUBLISHER EXAMPLE ---
        self.pub = self.create_publisher(Path, "/mission/waypoints", qos)

        path_msg = Path()
        path_msg.header.frame_id = "map"

        with open(csv_path, newline='') as f:
            reader = csv.reader(f)
            lat_lon = []
            lat = []
            lon = []
            x_y = []
            x = []
            y = []
            dist = []
            ranking = []
            for i, row in enumerate(reader):
                if len(row) < 2:
                    continue
                lat_lon.append([float(row[0]), float(row[1])])
                # lat.append(float(row[0]))
                # lon.append(float(row[1]))
                x_y.append([wgs84_to_enu(lat-1, lon[-1], lat0, lon0)])
                # x.append(wgs84_to_enu(lat[-1], lon[-1], lat0, lon0)[0])
                # y.append(wgs84_to_enu(lat[-1], lon[-1], lat0, lon0)[1])
                dist.append(math.dist((lat0, lon0), x_y[-1]))
                self.get_logger().info(f"Lat & Lon: {lat_lon[-1]}")
                self.get_logger().info(f"X & Y: {x_y[-1]}")
                self.get_logger().info(f"Distance: {dist}")
                
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            self.get_logger().info(f'Loaded waypoint {i}: lat={lat}, lon={lon} → x={x:.1f}, y={y:.1f}')

        self.pub.publish(path_msg)
        self.get_logger().info(colorStr(f"Published {len(path_msg.poses)} waypoints from {csv_path}", ColorCodes.GREEN_OK))


def main(args: list[str] | None = None) -> None:
    rclpy.init()
    node = WaypointManagerNode()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
