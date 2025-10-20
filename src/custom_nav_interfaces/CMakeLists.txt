cmake_minimum_required(VERSION 3.8)
project(custom_nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)        # 若 msg 里没引用可删

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/NavigationRequest.msg"
  "msg/NavigationResponse.msg"
  "msg/ActionConfig.msg"
  "msg/VoiceConfig.msg"
  "srv/NavigationExecute.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()