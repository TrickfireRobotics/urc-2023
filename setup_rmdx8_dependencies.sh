#!/usr/bin/env bash

# setup_motor_dependencies.sh
# Automatically sets up motor ROS2 dependencies

set -e  # Exit on any error

# Colors for output
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}[MOTOR SETUP] Setting up motor ROS2 dependencies...${NC}"

# Ensure we're in the project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# -----------------------
# Function to check command
# -----------------------
check_command() {
    CMD=$1
    NAME=$2
    INSTALL=$3

    if ! command -v "$CMD" &> /dev/null; then
        echo -e "${YELLOW}[MOTOR SETUP] $NAME not found.${NC}"
        echo -e "${BLUE}[MOTOR SETUP] Consider installing: $INSTALL${NC}"
        # Uncomment to auto-install system-wide
        # sudo apt-get install -y $INSTALL
    else
        echo -e "${GREEN}[MOTOR SETUP] $NAME found.${NC}"
    fi
}

# -----------------------
# Check essential commands
# -----------------------
check_command gcc "GCC compiler" "build-essential"
check_command g++ "G++ compiler" "build-essential"
check_command make "Make" "build-essential"
check_command cmake "CMake" "cmake"
check_command candump "CAN utils" "can-utils"
check_command ip "iproute2" "iproute2"

# -----------------------
# Check pybind11
# -----------------------
if python3 -m pybind11 --version &> /dev/null; then
    echo -e "${GREEN}[MOTOR SETUP] pybind11 found.${NC}"
else
    echo -e "${YELLOW}[MOTOR SETUP] pybind11 not found.${NC}"
    echo -e "${BLUE}[MOTOR SETUP] Installing system-wide pybind11...${NC}"
    # Uncomment to install
    sudo apt-get install -y pybind11-dev
fi

# -----------------------
# Check kernel CAN modules
# -----------------------
for mod in can can_raw can_dev; do
    if lsmod | grep -q "^$mod"; then
        echo -e "${GREEN}[MOTOR SETUP] Kernel module $mod loaded.${NC}"
    else
        echo -e "${YELLOW}[MOTOR SETUP] Kernel module $mod not loaded.${NC}"
        echo -e "${BLUE}[MOTOR SETUP] You may need: linux-modules-extra-$(uname -r)${NC}"
    fi
done


# Create src directory if it doesn't exist
mkdir -p src

cd src

# Setup myactuator_rmd
if [ ! -d "myactuator_rmd" ]; then
    echo -e "${BLUE}[MOTOR SETUP] Cloning myactuator_rmd...${NC}"
    git clone https://github.com/2b-t/myactuator_rmd.git
    cd myactuator_rmd
    git fetch --all --tags
    cd ..
    echo -e "${GREEN}[MOTOR SETUP] myactuator_rmd cloned successfully${NC}"
else
    echo -e "${YELLOW}[MOTOR SETUP] myactuator_rmd already exists, skipping...${NC}"
fi

echo -e "${GREEN}[MOTOR SETUP] Motor dependencies setup complete!${NC}"

# Return to original directory
cd "$SCRIPT_DIR"