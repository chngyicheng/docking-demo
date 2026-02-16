#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export RCUTILS_COLORIZED_OUTPUT=1
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger

tmux new-session -s turtlebot3\; \
	select-pane -T "Turtlebot3" \; send-keys "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" C-m \; \
	split-window -v \; select-pane -T "Navigation" \; send-keys "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~/map.yaml" C-m \; \
	select-pane -t 0 \; \
	split-window -h \; select-pane -T "Localisation" \; send-keys "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True" C-m \; \
	select-pane -t 2 \; \
	split-window -v \; select-pane -T "Teleops" \; send-keys "ros2 run turtlebot3_teleop teleop_keyboard" C-m \; \
	select-layout tile ";"

