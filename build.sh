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

SO_FILE=$(find install/myactuator_rmd/local/lib/python3.10/dist-packages/myactuator_rmd/ -name "myactuator_rmd_py*.so" 2>/dev/null)

if [ -n "$SO_FILE" ]; then
    echo -e "${BLUE}[MOTOR SETUP] Fixing myactuator_rmd_py install path...${NC}"
    cp "$SO_FILE" install/myactuator_rmd/local/lib/python3.10/dist-packages/
    echo -e "${GREEN}[MOTOR SETUP] Copied $(basename $SO_FILE) to dist-packages root${NC}"
else
    echo -e "${YELLOW}[MOTOR SETUP] myactuator_rmd_py .so not found in expected location, skipping fix${NC}"
fi

colcon build \
    --symlink-install \
    --base-paths /home/trickfire/urc-2023\
    --packages-ignore myactuator_rmd \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Debug
