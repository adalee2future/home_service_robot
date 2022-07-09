#!/bin/sh

source devel/setup.bash

xterm  -e  "roslaunch my_robot world.launch" &
sleep 5

xterm  -e  "roslaunch my_robot amcl.launch" & 
sleep 5

xterm  -e  "rosrun rviz rviz -d src/rvizConfig/navigation.rviz" & 
sleep 5

xterm  -e  "roslaunch pick_objects pick_objects.launch"
#roslaunch pick_objects pick_objects.launch

