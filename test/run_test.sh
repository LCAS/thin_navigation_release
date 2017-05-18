#!/bin/bash

xterm -e "roslaunch thin_navigation map.launch" &
sleep 3
xterm -e "roslaunch thin_navigation robot.launch" &
sleep 3
xterm -e "rosrun rviz rviz -d robot_0.rviz" &

