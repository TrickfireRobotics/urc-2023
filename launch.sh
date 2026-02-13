#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/install/setup.bash"

#modprobe can
#modprobe can_raw
#modprobe mttcan
#ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
#ip link set can0 up

# Launch octomap server
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/filtered_point_cloud \
  -p resolution:=0.05 \
  -p frame_id:=map \
  -p latch:=true \
  -p publish_2d_map:=true \
  -p visualize_free_space:=true &

# Add to the python import pathes
export PYTHONPATH="$SCRIPT_DIR/src/:$PYTHONPATH"

ros2 launch viator_launch robot.launch.py


