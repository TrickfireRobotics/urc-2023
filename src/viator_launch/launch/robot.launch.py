import os

import launch
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

can_moteus_node = Node(package="can_moteus", executable="can_moteus", name="can_moteus_node")


drivebase_node = Node(package="drivebase", executable="drivebase", name="drivebase_node")

can_rmdx8_node = Node(package="can_rmdx8", executable="can_rmdx8", name="can_rmdx8_node")

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
cmd_vel_to_wheel_vel_node = Node(
    package="autonomous_nav", executable="cmd_vel_to_wheel_vel", name="cmd_vel_to_wheel_vel"
)
nav2_params = os.path.join(
    get_package_share_directory("autonomous_nav"), "config", "nav2_params.yaml"
)
controller_server_node = Node(
    package="nav2_controller",
    executable="controller_server",
    name="controller_server",
    output="screen",
    parameters=[nav2_params],
)
global_costmap_node = Node(
    package="nav2_costmap_2d",
    executable="nav2_costmap_2d",
    name="global_costmap",
    parameters=[nav2_params],
)

# Nav2 lifecycle manager - manages controller_server lifecycle states
lifecycle_manager_node = Node(
    package="nav2_lifecycle_manager",
    executable="lifecycle_manager",
    name="lifecycle_manager_navigation",
    output="screen",
    parameters=[
        {
            "autostart": True,
            "node_names": ["controller_server"],
        }
    ],
)
#vision_processing_node = Node(
#    package="autonomous_nav", executable="vision_processing_node", name="vision_processing_node"
#)

# Include the ZED camera launch file from zed_wrapper
zed_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py")
    ),
    launch_arguments={"camera_model": "zed2i", "composable_node": "False"}.items(),
)

ublox_config_path = os.path.join(
    get_package_share_directory("autonomous_nav"), "config", "ublox.yaml"
)
gps_node = Node(
    package="ublox_gps",
    executable="ublox_gps_node",
    name="gps_node",
    output="screen",
    parameters=[
        ublox_config_path,
        {
            "frame_id": "gps",
            "dynamic_model": "automotive",  # better for a wheeled rover than "portable"
            "enable_pps": True,  # keep if you wire/use PPS
            "tmode3": 0,  # 0 = rover mode
            # RTK/NTRIP (keep only if you actually use a caster):
            "rtcm_caster_address": "3.143.243.81",
            "rtcm_caster_port": 2101,
            "mount_point": "BOTHWA",
            "username": "jakek927@gmail.com",
            "password": "none",
            "publish_rtcm": True,
        },
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
static_zed_base_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="base_link_tf_broadcaster",
    arguments=["0", "0", "0", "0", "0", "0", "1", "zed_camera_link", "base_link"],
)


static_zed_gps_tf = Node(
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
            can_rmdx8_node,
            mission_control_updater_node,
            arm_node,
            navsat_transform,
            navigation_node,
            ekf_node,
            static_zed_gps_tf,
            static_zed_base_tf,
            global_costmap_node,
            heartbeat_node,
            camera_node,
            gps_anchor_node,
            sensor_processing_node,
            cmd_vel_to_wheel_vel_node,
            launch_include,
            gps_node,
            zed_launch,
            # Start lifecycle manager first with minimal delay
            TimerAction(period=1.0, actions=[lifecycle_manager_node]),
            # Then start controller_server after delay to allow tf frames to be published
            TimerAction(period=5.0, actions=[controller_server_node]),
        ]
    )
