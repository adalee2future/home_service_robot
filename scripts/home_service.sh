#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE="map/ada.world"

source devel/setup.bash

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10

xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" & 
sleep 10

xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 10

xterm -e "rosrun add_markers add_markers" &
xterm -e "rosrun pick_objects pick_objects" 
