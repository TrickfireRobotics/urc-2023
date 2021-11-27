import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    robot_container = ComposableNodeContainer(
        name='robot',
        package='rclcpp_components',
        namespace='',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hello_world',
                plugin='hello_world::hello_node',
                name='hello_node',
            ),
        ],
        output='screen',
        emulate_tty=True
    )

    return launch.LaunchDescription([
        robot_container
    ])