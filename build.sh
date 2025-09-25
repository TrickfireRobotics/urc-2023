#!/usr/bin/env bash

# Run ZED dependency setup first
./setup_zed_dependencies.sh

source /opt/ros/$ROS_DISTRO/setup.bash

# build zed components first
colcon build \
--symlink-install \
--base-paths /home/trickfire/jake-voxel-urc/urc-2023 \
--packages-select zed_components \
--cmake-args \
-DCMAKE_BUILD_TYPE=Debug

colcon build \
--symlink-install \
--base-paths /home/trickfire/jake-voxel-urc/urc-2023 \
--packages-select zed_wrapper \
--cmake-args \
-DCMAKE_BUILD_TYPE=Debug

colcon build \
    --symlink-install \
    --base-paths /home/jake-voxel-urc/urc-2023\
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug
