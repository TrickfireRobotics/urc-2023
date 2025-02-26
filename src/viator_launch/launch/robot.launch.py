import os

import launch
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

can_moteus_node = Node(package="can_moteus", executable="can_moteus", name="can_moteus_node")


drivebase_node = Node(package="drivebase", executable="drivebase", name="drivebase_node")


mission_control_updater_node = Node(
    package="mission_control_updater",
    executable="mission_control_updater",
    name="mission_control_updater_node",
)

arm_node = Node(package="arm", executable="arm", name="arm_node")

heartbeat_node = Node(package="heartbeat", executable="heartbeat", name="heartbeat_node")

camera_node = Node(package="camera", executable="roscamera", name="camera_node")

# This is the example node. It will show ROS timers, subscribers, and publishers
# To include it in the startup, add it to the array in the generate_launch_description() method
example_node = Node(package="example_node", executable="myExampleNode", name="my_example_node")

launch_include = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("rosbridge_server"), "launch/rosbridge_websocket_launch.xml"
        ),
    ),
    launch_arguments=[("use_compression", "true")],
)

# Autonomous navigation nodes
decision_making_node = Node(
    package="autonomous_nav", executable="decision_making_node", name="decision_making_node"
)
gps_anchor_node = Node(
    package="autonomous_nav", executable="gps_anchor_node", name="gps_anchor_node"
)
navigation_node = Node(
    package="autonomous_nav", executable="navigation_node", name="navigation_node"
)
sensor_processing_node = Node(
    package="autonomous_nav", executable="sensor_processing_node", name="sensor_processing_node"
)

# Include the ZED camera launch file from zed_wrapper
zed_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py")
    ),
    launch_arguments={"camera_model": "zed2i", "composable_node": "False"}.items(),
)

gps_node = Node(
    package="ublox_gps",
    executable="ublox_gps_node",
    name="gps_node",
    output="screen",
    parameters=[
        {
            "frame_id": "gps",
            "rate": 4.0,  # GNSS data update rate in Hz
            "dynamic_model": "portable",  # Model for stationary/moving applications
            "nav_rate": 1,  # Must be 1 Hz for HPG Ref devices
            "enable_pps": True,  # Enable Pulse-Per-Second (PPS) if needed
            "tmode3": 0,  # Set to Rover mode (Instead of fixed mode)
            # RTK/NTRIP settings for RTK2Go
            "rtcm_caster_address": "3.143.243.81",
            "rtcm_caster_port": 2101,
            "mount_point": "BOTHWA",  # Case-sensitive!
            "username": "jakek927@gmail.com",  # Replace with your actual email
            "password": "none",  # Required but ignored
            "publish_rtcm": True,  # Ensure RTCM corrections are published
        }
    ],
)

# Path to your navsat_transform config
navsat_config_path = os.path.join(
    get_package_share_directory("autonomous_nav"), "config", "navsat_transform.yaml"
)

navsat_transform = Node(
    package="robot_localization",
    executable="navsat_transform_node",
    name="navsat_transform_node",
    output="screen",
    # Remap /imu -> /zed/zed_node/imu/data
    # Remap /gps/fix -> /fix
    remappings=[
        ("/imu", "/zed/zed_node/imu/data"),
        ("/gps/fix", "/fix"),  # if needed
    ],
    parameters=[navsat_config_path],  # Load config
)

# Path to your configuration YAML for localization ekf_node
config_file_path = os.path.join(get_package_share_directory("autonomous_nav"), "config", "ekf.yaml")

ekf_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node",
    output="screen",
    parameters=[config_file_path],
)


static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="gps_tf_broadcaster",
    arguments=["-0.45", "-0.12", "0", "0", "0", "0", "1", "zed_camera_link", "gps"],
)


def generate_launch_description() -> launch.LaunchDescription:  # pylint: disable=invalid-name
    return launch.LaunchDescription(
        [
            can_moteus_node,
            drivebase_node,
            mission_control_updater_node,
            arm_node,
            heartbeat_node,
            camera_node,
            decision_making_node,
            gps_anchor_node,
            navigation_node,
            sensor_processing_node,
            launch_include,
            gps_node,
            zed_launch,
            navsat_transform,
            ekf_node,
            static_tf,
        ]
    )
