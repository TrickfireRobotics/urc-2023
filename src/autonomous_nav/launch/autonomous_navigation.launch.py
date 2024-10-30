from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="autonomous_nav_pkg",
                executable="navigation_node",
                name="navigation_node",
                output="screen",
                parameters=["config/parameters.yaml"],  # Optional: Load parameters if needed
            ),
            Node(
                package="autonomous_nav_pkg",
                executable="sensor_processing_node",
                name="sensor_processing_node",
                output="screen",
            ),
            Node(
                package="autonomous_nav_pkg",
                executable="control_node",
                name="control_node",
                output="screen",
            ),
            Node(
                package="autonomous_nav_pkg",
                executable="localization_node",
                name="localization_node",
                output="screen",
            ),
            Node(
                package="autonomous_nav_pkg",
                executable="decision_making_node",
                name="decision_making_node",
                output="screen",
            ),
        ]
    )
