#!/bin/bash
# 用法：
#   ./launch_all.sh              # 启动全部进程
#   ./launch_all.sh 1            # 启动 dog_control
#   ./launch_all.sh 2 wp.yaml    # 启动 btree（必须带 waypoints 文件）
set -euo pipefail

map_yaml='/home/wya/projects/slam/mapping/maps/map002.yaml'
map_pcd='/home/wya/projects/slam/mapping/maps/map002_scans_voxel.pcd'
db=/tmp/run_test.name.pid

launch(){
  local name=$1; shift
  gnome-terminal --tab -t "$name" -- bash -c "
set +m
$* &                               # 启动节点（后台）
echo \"$name \$\$\" >> \"$db\"     # 进程名字 + 组长PID
wait                               # 等节点真正结束
exec bash" &
}

# 不带参数：启动全部
if [[ $# -eq 0 ]]; then
  > "$db"   # 清空旧记录
  launch nav2 \
    "ros2 launch dog_guide guide.launch.py map:=$map_yaml"
  sleep 0.5s
  launch dog_control \
    "ros2 run dog_guide dog_controller"
  sleep 0.5s
  launch livox_lidar \
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
  sleep 1s
  launch fast-lio2-loc \
    "ros2 launch fast_lio_localization localization.launch.py pcd_map_topic:=cloud_pcd map:=${map_pcd}"

  echo "已启动节点，记录文件：$db"
  exit 0
fi

# 带参数：启动单个进程
case ${1:-} in
  1) launch dog_control \
       "ros2 run dog_guide dog_controller" ;;
  2) if [[ -z ${2:-} ]]; then
       echo "错误：启动 btree 必须提供 waypoints 文件！例：$0 5 wp.yaml" >&2
       exit 1
     fi
     launch btree \
       "ros2 run dog_guide bt_creater --ros-args -p waypoints_file:=$(realpath "$2")" ;;
  *) echo "用法：$0 [1-2 [waypoints]]" >&2; exit 1 ;;
esac
echo "已启动节点，记录文件：$db"