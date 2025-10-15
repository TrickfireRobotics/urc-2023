#!/usr/bin/env bash

# setup_zed_dependencies.sh
# Automatically sets up ZED ROS2 dependencies

set -e  # Exit on any error

# Colors for output
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}[ZED SETUP] Setting up ZED ROS2 dependencies...${NC}"

# Ensure we're in the project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Create src directory if it doesn't exist
mkdir -p src

cd src

# Setup zed-ros2-wrapper
if [ ! -d "zed-ros2-wrapper" ]; then
    echo -e "${BLUE}[ZED SETUP] Cloning zed-ros2-wrapper...${NC}"
    git clone https://github.com/stereolabs/zed-ros2-wrapper.git
    echo -e "${GREEN}[ZED SETUP] zed-ros2-wrapper cloned successfully${NC}"
else
    echo -e "${YELLOW}[ZED SETUP] zed-ros2-wrapper already exists, skipping...${NC}"
fi

# Setup zed-ros2-examples (optional)
if [ ! -d "zed-ros2-examples" ]; then
    echo -e "${BLUE}[ZED SETUP] Cloning zed-ros2-examples...${NC}"
    git clone https://github.com/stereolabs/zed-ros2-examples.git
    echo -e "${GREEN}[ZED SETUP] zed-ros2-examples cloned successfully${NC}"
else
    echo -e "${YELLOW}[ZED SETUP] zed-ros2-examples already exists, skipping...${NC}"
fi

echo -e "${GREEN}[ZED SETUP] ZED dependencies setup complete!${NC}"

# Return to original directory
cd "$SCRIPT_DIR"