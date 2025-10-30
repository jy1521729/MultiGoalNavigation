#!/bin/bash
net_name='wlp170s0f0'   # 修改为你的网络接口名称, 例如'wlp9s0'

export ROS_DOMAIN_ID=10
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS xmlns="https://cdds.io/config"><Domain id="0"><General><Interfaces><NetworkInterface name="'${net_name}'" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast><EnableMulticastLoopback>true</EnableMulticastLoopback></General></Domain></CycloneDDS>'

ros2 run nav_command nav_server
