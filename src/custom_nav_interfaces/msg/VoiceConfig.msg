# VoiceConfig.msg
# 语音配置消息定义

# 语音ID
string voice_id           # 语音ID

# 语音内容
string voice_content      # 语音播放内容

# 播放参数
float64 play_duration     # 播放时长 (秒)
float64 play_delay         # 播放前等待时间 (秒) - 站立多少秒后开始播放语音

# 音频文件传输
bool has_audio_file       # 是否包含音频文件数据
string audio_file_name    # 音频文件名称 (包含扩展名，如 "voice_001.wav")
string audio_file_format  # 音频文件格式 (wav, mp3, etc.)
uint8[] audio_file_data   # 音频文件二进制数据
uint32 audio_file_size    # 音频文件大小 (字节)