#!/bin/bash
# 用法:
#   ./kill_run_test.sh        # 杀全部
#   ./kill_run_test.sh nav2   # 只杀 nav2
#   ./kill_run_test.sh nav* bt*  # 支持通配

db=/tmp/run_test.name.pid
timeout=5        # 等待秒数
code=0           # 最终返回码：0 全部成功，1 有残留

[[ -s $db ]] || { echo "无记录可杀"; exit 0; }

patterns=("$@")
(( ${#patterns[@]} )) || patterns=('*')

#-------- 杀整组（含子孙）的辅助函数 --------
killgroup(){
  local sig=$1 pid=$2
  # 杀进程组：负号
  kill -$sig -"$pid" 2>/dev/null
}

#-------- 等待消失 --------
waitgone(){
  local pid=$1
  local t=0
  while (( t < timeout )); do
    [[ -d /proc/$pid ]] || return 0   # 已死
    sleep 0.5
    ((t++))
  done
  return 1                            # 超时仍未死
}

#-------- 主循环 --------
while read -r name pid; do
  matched=0
  for pat in "${patterns[@]}"; do
    [[ $name == $pat ]] && { matched=1; break; }
  done
  ((matched)) || continue

  # 1. SIGTERM
  killgroup TERM "$pid"
  if waitgone "$pid"; then
    echo "[$name] 进程组 $pid 已退出"
    sed -i "\|^$name $pid$|d" "$db"
    continue
  fi

  # 2. SIGKILL
  echo "[$name] 超时，强杀进程组 $pid"
  killgroup KILL "$pid"
  if waitgone "$pid"; then
    echo "[$name] 进程组 $pid 已强杀"
    sed -i "\|^$name $pid$|d" "$db"
  else
    echo "[$name] 进程组 $pid 仍存活！"
    code=1        # 标记失败
  fi
done < "$db"

exit $code