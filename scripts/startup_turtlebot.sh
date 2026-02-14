#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/docking-demo/install/setup.bash

export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger

tmux new-session \; \
	select-pane -T "Turtlebot3" \; send-keys "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" C-m \; \
	split-window -v \; select-pane -T "Navigation" \; send-keys "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml" C-m \; \
	select-pane -t 0 \; \
	split-window -h \; select-pane -T "Localisation" \; send-keys "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True" C-m \; \
	select-pane -t 2 \; \
	split-window -v \; select-pane -T "Teleops" \; send-keys "ros2 run turtlebot3_teleop teleop_keyboard" C-m \; \
	select-layout tile ";"

