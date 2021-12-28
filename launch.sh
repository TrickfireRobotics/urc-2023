#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/trickfire/NasaRmc2022/install/setup.bash

ros2 launch houdini_launch robot.launch.py
