#!/bin/sh

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot world.launch" &

sleep 1

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot amcl.launch" & 

sleep 1

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5 # keeping large to enable visualization

x-terminal-emulator -e "cd $(pwd)/../..;
rosparam load $(pwd)/src/config/marker_config.yaml;
rosrun pick_objects pick_objects;
zsh" &