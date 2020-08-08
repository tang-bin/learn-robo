#!/bin/sh
x-terminal-emulator -e "cd $(pwd)/../..;
source devel/setup.zsh" &

sleep 1
x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot world.launch" &

sleep 5
x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5
x-terminal-emulator -e "cd $(pwd)/../..;
roslaunch my_robot amcl.launch" &

sleep 1
x-terminal-emulator -e "cd $(pwd)/../..;
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.5 _turn:=0.5" &
# x-terminal-emulator -e " roslaunch my_robot teleop.launch" &