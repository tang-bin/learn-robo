#!/bin/sh

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot world.launch" & 

sleep 5

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot amcl.launch" &

sleep 5

x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch add_markers home_service_rviz_config.launch rviz_config_file:=$(pwd)/../rvizConfig/home_service.rviz" &

sleep 5

x-terminal-emulator -e "cd $(pwd)/../..;
rosparam load $(pwd)/../config/marker_config.yaml;
rosrun add_markers add_markers" &