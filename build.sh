#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run rmxd8 dependency setup first
"$SCRIPT_DIR/setup_rmdx8_dependencies.sh"
#build motor components
colcon build \
    --symlink-install \
    --base-paths "$SCRIPT_DIR"\
    --packages-select myactuator_rmd \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DPYTHON_BINDINGS=ON


colcon build \
    --symlink-install \
    --base-paths /home/trickfire/urc-2023\
    --packages-ignore myactuator_rmd \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug
