#!/bin/bash
xterm -e "roslaunch thin_navigation map.launch" &
sleep 3

xterm -e "roslaunch thin_navigation robot.launch robotname:=robot_0" &
sleep 3
xterm -e "roslaunch thin_navigation robot.launch robotname:=robot_1 initpose_x:=-2.5 initpose_y:=1.8 initpose_th_rad:=0.0" &
sleep 3
xterm -e "rosrun rviz rviz -d robot_0.rviz" &
sleep 3
xterm -e "rosrun rviz rviz -d robot_1.rviz" &


