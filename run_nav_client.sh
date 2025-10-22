#!/bin/bash

api_id=${1:-1001}          # 取第一个参数，缺省 1001
[[ $api_id =~ ^(10[0-2][1-5])$ ]] || { echo "invalid params"; exit 1; }
echo "api_id: ${api_id}"

source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS xmlns="https://cdds.io/config"><Domain id="0"><General><Interfaces><NetworkInterface name="wlp9s0" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast><EnableMulticastLoopback>true</EnableMulticastLoopback></General></Domain></CycloneDDS>'

ros2 run nav_command nav_client --ros-args -p api_id:=${api_id}
