#!/bin/bash

# Script to install MoveIt 2 dependencies for ROS 2 project
# Assumes ROS 2 Humble is installed; adjust distro if needed (e.g., jazzy)

set -e  # Exit on error

echo "Updating package list..."
sudo apt update

echo "Installing ROS 2 MoveIt 2 core packages..."
sudo apt install -y ros-humble-moveit2

echo "Installing MoveIt Python API..."
sudo apt install -y ros-humble-moveit-py

echo "Installing ROS 2 controllers for execution..."
sudo apt install -y ros-humble-ros2-control ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller

echo "Installing optional visualization tools (RViz and MoveIt RViz)..."
sudo apt install -y ros-humble-rviz2 ros-humble-moveit-rviz

echo "Installation complete. Source your ROS 2 setup: source /opt/ros/humble/setup.bash"
echo "Then build your workspace: ./build.sh"
