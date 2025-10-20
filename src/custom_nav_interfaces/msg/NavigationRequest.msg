# NavigationRequest.msg
# 导航任务请求消息

# 请求头信息
uint64 request_id          # 请求ID，用于追踪请求
uint32 api_id             # API ID，标识接口类型（1001: 启动, 1002: 紧急停止）

# 路线信息
string route_id           # 路线ID
string route_name         # 路线名称

# 路径点数组
Waypoint[] waypoints      # 路径点列表