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
control_node = Node(package="autonomous_nav", executable="control_node", name="control_node")
decision_making_node = Node(
    package="autonomous_nav", executable="decision_making_node", name="decision_making_node"
)
localization_node = Node(
    package="autonomous_nav", executable="localization_node", name="localization_node"
)
navigation_node = Node(
    package="autonomous_nav", executable="navigation_node", name="navigation_node"
)
sensor_processing_node = Node(
    package="autonomous_nav", executable="sensor_processing_node", name="sensor_processing_node"
)

gps_node = Node(
    package="ublox_gps",
    executable="ublox_gps_node",
    name="gps_node",
    output="screen",
    parameters=[
        {
            "frame_id": "gps",
            "rate": 4.0,                # GNSS data update rate in Hz
            "dynamic_model": "portable",  # Model for stationary/moving applications
            "nav_rate": 1,              # Must be 1 Hz for HPG Ref devices
            "enable_pps": True,         # Enable Pulse-Per-Second (PPS) if needed
            "tmode3": {
                "enabled": True,        # Must be enabled for HPG devices
                "mode": 0,              # 0 is a common default mode (survey-in disabled)
                "flags": 0,             # No flags set
                "reserved0": 0,         # Reserved field (default 0)
                "reserved1": 0,         # Reserved field (default 0)
                "ecefX": 0.0,           # ECEF X coordinate (set appropriately if needed)
                "ecefY": 0.0,           # ECEF Y coordinate
                "ecefZ": 0.0,           # ECEF Z coordinate
                "posAcc": 1000.0,       # Position accuracy (in mm, adjust as needed)
            },
        }
    ],
)


def generate_launch_description() -> launch.LaunchDescription:  # pylint: disable=invalid-name
    # Include the ZED camera launch file from zed_wrapper
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )
    
    return launch.LaunchDescription(
        [
            can_moteus_node,
            drivebase_node,
            mission_control_updater_node,
            arm_node,
            heartbeat_node,
            camera_node,
            control_node,
            decision_making_node,
            localization_node,
            navigation_node,
            sensor_processing_node,
            launch_include,
            gps_node,
            zed_launch,
        ]
    )
