#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Run ZED dependency setup first
"$SCRIPT_DIR/setup_zed_dependencies.sh"

source /opt/ros/$ROS_DISTRO/setup.bash

# build zed components first
colcon build \
--symlink-install \
--base-paths "$SCRIPT_DIR" \
--packages-select zed_components \
--cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon build \
--symlink-install \
--base-paths "$SCRIPT_DIR" \
--packages-select zed_wrapper \
--cmake-args \
-DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon build \
    --symlink-install \
    --base-paths "$SCRIPT_DIR"\
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo

