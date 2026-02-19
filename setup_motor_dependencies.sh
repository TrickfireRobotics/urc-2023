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