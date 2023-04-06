import launch
from launch_ros.actions import ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

hello_node = ComposableNode(
    package='hello_world',
    plugin='hello_world::hello_node',
    name='hello_node'
)

mission_control_node = Node(
    package='mission_control',
    executable='mission_control',
    name='mission_control_node'
)
# Composable Nodes launched in a Composable Node container will share a process
# and can use very fast inter-process communication instead of publishing
# messages over a network socket.
# Note: "Composable Node container" does not mean "Docker-like container".
robot_container = ComposableNodeContainer(
    name='robot',
    package='rclcpp_components',
    namespace='',
    executable='component_container',
    composable_node_descriptions=[
        hello_node
    ],
    output='screen',
    emulate_tty=True
)


def generate_launch_description():
    ld = launch.LaunchDescription()
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 
                         'launch/rosbridge_websocket_launch.xml')))
    ld.add_action(robot_container)
    ld.add_action(mission_control_node)
    ld.add_action(rosbridge_launch)
    return ld
