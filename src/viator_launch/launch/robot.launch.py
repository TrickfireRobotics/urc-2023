import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

hello_node = ComposableNode(
    package='hello_world',
    plugin='hello_world::hello_node',
    name='hello_node'
)

can_moteus_node = Node(
    package='can_moteus',
    executable='can_moteus',
    name='can_moteus_node'
)

robot_info_node = Node(
    package='robot_info',
    executable='listener',
    name='TestSubscriber'
)

robot_info_node_talker = Node(
    package='robot_info',
    executable='talker',
    name='TestPublisher'
)

drivebase_node = Node(
    package='drivebase',
    executable='drivebase',
    name='drivebase_node'
)

testing_node = Node(
    package='testingInterface',
    executable='testingNode',
    name='testing_node'
)



# # Composable Nodes launched in a Composable Node container will share a process
# # and can use very fast inter-process communication instead of publishing
# # messages over a network socket.
# # Note: "Composable Node container" does not mean "Docker-like container".
# robot_container = ComposableNodeContainer(
#     name='robot',
#     package='rclcpp_components',
#     namespace='',
#     executable='component_container',
#     composable_node_descriptions=[
#         hello_node
#     ],
#     output='screen',
#     emulate_tty=True
# )


def generate_launch_description():
    return launch.LaunchDescription([
        can_moteus_node,
        drivebase_node
    ])