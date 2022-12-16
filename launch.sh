#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/trickfire/urc-2023/install/setup.bash \

echo "ADAMMMMMMMMMMMMMMMMMMMMMM"
ros2 launch rosbridge_server rosbridge_websocket.launch
ros2 launch viator_launch robot.launch.py