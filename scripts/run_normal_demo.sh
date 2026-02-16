#!/bin/bash
source /opt/ros/humble/setup.bash
#source ~/<your_ros2_ws>/install/setup.bash

export ROS_DOMAIN_ID=30
export ENABLE_SCAN_DROPOUT=false
export ENABLE_SCAN_CORRUPTION=false

tmux new-session -s docking-demo \; \
	select-pane -T "Docking" \; send-keys "export RCUTILS_COLORIZED_OUTPUT=1 && ros2 launch docking_demo docking_demo.launch.py enable_scan_dropout:=${ENABLE_SCAN_DROPOUT} enable_scan_corruption:=${ENABLE_SCAN_CORRUPTION}" C-m \; \
