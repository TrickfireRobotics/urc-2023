"""
Launch file for the autonomous_nav package.

This launch file starts all the nodes required for autonomous navigation, 
including control, decision-making, navigation, and sensor processing nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the autonomous navigation system.

    This function defines the nodes to be launched as part of the autonomous_nav package.
    """
    return LaunchDescription(
        [
            Node(
                package="autonomous_nav",
                executable="control_node",
                name="control_node",
                output="screen",
                parameters=[],
            ),
            Node(
                package="autonomous_nav",
                executable="decision_making_node",
                name="decision_making_node",
                output="screen",
                parameters=[],
            ),
            Node(
                package="autonomous_nav",
                executable="navigation_node",
                name="navigation_node",
                output="screen",
                parameters=[],
            ),
            Node(
                package="autonomous_nav",
                executable="sensor_processing_node",
                name="sensor_processing_node",
                output="screen",
                parameters=[],
            ),
        ]
    )
