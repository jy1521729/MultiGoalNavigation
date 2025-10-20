#!/bin/bash

waypoints=''
if [ -n "$1" ]; then
  waypoints='waypoints:='$1
fi

map_yaml='/home/wya/projects/slam/mapping/maps/map002.yaml'
map_pcd='/home/wya/projects/slam/mapping/maps/map002_scans_voxel.pcd'

echo 'params: '${waypoints}

{
gnome-terminal --tab -t "dog_guide" -- bash -c "ros2 launch dog_guide guide.launch.py map:=${map_yaml} ${waypoints};exec bash"
}&

sleep 1s

{
gnome-terminal --tab -t "dog_control" -- bash -c "ros2 run dog_guide dog_controller;exec bash"
}&

sleep 1s

{
gnome-terminal --tab -t "livox_lidar" -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py;exec bash"
}&
 
sleep 2s

{
gnome-terminal --tab -t "fast-lio2-loc" -- bash -c "ros2 launch fast_lio_localization localization.launch.py pcd_map_topic:=cloud_pcd map:=${map_pcd};exec bash"
}&

sleep 5s

{
gnome-terminal --tab -t "pub_initial_pose" -- bash -c "ros2 run fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0;exec bash"
}&
