#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/setup.bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source "$SCRIPT_DIR/install/setup.bash"

chmod +x ./speed_cansend.sh
./speed_cansend.sh

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

# Kill all child processes when Ctrl-C is pressed
trap "echo 'Stopping...'; kill 0" SIGINT

ros2 launch viator_launch robot.launch.py &

#activate vision packages
source vision_packages2/bin/activate

#apperantly this will still launch vision processing as a fully working node
python3 ~/autonomous-nav-vision/urc-2023/src/autonomous_nav/autonomous_nav/vision_processing_node.py

wait
