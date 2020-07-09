#!/bin/sh
x-terminal-emulator -e " gazebo " &
sleep 5
x-terminal-emulator -e " source /opt/ros/noetic/setup.zsh; roscore" &
sleep 5
x-terminal-emulator -e " rosrun rviz rviz"
