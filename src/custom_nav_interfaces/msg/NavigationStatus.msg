# NavigationStatus.msg
# 导航状态消息

string status               # Current status: "idle", "starting", "running", "completed", "error", "emergency", "stopped"
string phase                # Current phase: "localizing", "navigating", "voice_playing"
string error_code           # Error code if status is error
string error_message        # Detailed error message
bool device_connect         # Whether device is connected
bool localization_ready     # Whether localization is ready
bool navigation_ready       # Whether navigation system is ready