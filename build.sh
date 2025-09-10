#!/usr/bin/env bash

# Run ZED dependency setup first
./setup_zed_dependencies.sh

source /opt/ros/$ROS_DISTRO/setup.bash

colcon build \
    --symlink-install \
    --base-paths /home/jake-voxel-urc/urc-2023\
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug
