# Waypoint.msg
# 路径点消息定义

# 路径点基本信息
string waypoint_id        # 路径点ID

# 地图坐标系位置信息
float64 x
float64 y
float64 z
float64 yaw

# 动作配置
ActionConfig action       # 动作配置

# 语音配置
VoiceConfig voice         # 语音配置