#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/trickfire/NasaRmc2022/install/setup.bash

colcon test \
    --base-paths /home/trickfire/NasaRmc2022

colcon test-result \
    --test-result-base /home/trickfire/NasaRmc2022 \
    --verbose
