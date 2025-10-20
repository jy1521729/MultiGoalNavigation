#!/bin/bash

waypoints=''
if [ -n "$1" ]; then
  waypoints='waypoints:='$1
fi

map_yaml='/home/ubuntu/code/xjy/xjy_work/datasets/maps/house_map.yaml'

{
gnome-terminal --tab -t "rviz" -- bash -c "rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz;exec bash"
}&

sleep 1s

{
gnome-terminal --tab -t "dog_guide" -- bash -c "ros2 launch dog_guide guide.launch.py map:=${map_yaml} ${waypoints};exec bash"
}&
