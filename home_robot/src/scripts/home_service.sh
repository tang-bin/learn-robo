#!/bin/sh

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot world.launch" &

sleep 1

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot amcl.launch" &

sleep 1
x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch add_markers home_service_rviz_config.launch rviz_config_file:=$(pwd)/src/rvizConfig/home_service.rviz" &

sleep 5
x-terminal-emulator -e "cd $(pwd)/../..;
rosparam load $(pwd)/src/config/marker_config.yaml;
rosrun add_markers add_markers " &

sleep 2
x-terminal-emulator -e "cd $(pwd)/../..;
rosparam load $(pwd)/src/config/marker_config.yaml;
rosrun pick_objects pick_objects" &