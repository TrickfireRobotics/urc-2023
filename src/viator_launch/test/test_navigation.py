import os
import sys
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import rclpy

# make the test description


def generate_test_description():
    file_path = os.path.dirname(__file__)
    navigation_node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[
            os.path.join(file_path, ".. ..", "autonomous_nav/autonomous_nav", "navigation_node.py")
        ],
        additional_env={"PYTHONUNBUFFERED": 1},
        parameters=[{}],
    )
