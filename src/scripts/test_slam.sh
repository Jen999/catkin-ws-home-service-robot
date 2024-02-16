#!/bin/sh
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_gazebo turtlebot3_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot3_slam turtlebot3_slam.launch " &
sleep 5
xterm -e " roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch "
