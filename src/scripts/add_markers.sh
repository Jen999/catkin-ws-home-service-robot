#!/bin/sh
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_gazebo turtlebot3_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot3_navigation turtlebot3_navigation.launch " &
sleep 5
xterm -e " rosrun add_markers add_markers "