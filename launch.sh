#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/trickfire/urc-2023/install/setup.bash

ros2 launch viator_launch robot.launch.py & ros2 launch rosbridge_server rosbridge_websocket_launch.xml
