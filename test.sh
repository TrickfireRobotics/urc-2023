#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/trickfire/urc-2023/install/setup.bash

colcon test \
    --base-paths /home/trickfire/urc-2023

colcon test-result \
    --test-result-base /home/trickfire/urc-2023 \
    --verbose
