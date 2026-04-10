#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# Run ZED dependency setup first
chmod +x "$SCRIPT_DIR/setup_zed_dependencies.sh"
"$SCRIPT_DIR/setup_zed_dependencies.sh"

# Run rmxd8 dependency setup first
chmod +x "$SCRIPT_DIR/setup_rmdx8_dependencies.sh"
"$SCRIPT_DIR/setup_rmdx8_dependencies.sh"

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
    --base-paths "$SCRIPT_DIR"\
    --packages-ignore myactuator_rmd \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo

