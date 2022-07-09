#!/bin/sh

source devel/setup.bash

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10

xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 10

xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

