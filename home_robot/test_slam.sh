#!/bin/sh
x-terminal-emulator -e " source devel/setup.zsh" &
sleep 1
x-terminal-emulator -e " roslaunch my_robot world.launch" &
sleep 5
x-terminal-emulator -e " roslaunch my_robot mapping.launch" &
sleep 5
x-terminal-emulator -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
# x-terminal-emulator -e " roslaunch my_robot amcl.launch" &
x-terminal-emulator -e " roslaunch turtlebot_gazebo amcl_demo" &
sleep 1
x-terminal-emulator -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.1 _turn:=1.0" &
# x-terminal-emulator -e " roslaunch my_robot teleop.launch" &