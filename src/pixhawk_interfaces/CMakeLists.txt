cmake_minimum_required(VERSION 3.8)
project(pixhawk_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionItem.msg"
  "msg/AvoidanceAdvisory.msg"
  "srv/MissionDownload.srv"
  "srv/MissionUpload.srv"
  "srv/CommandLong.srv"
  "srv/ParamGet.srv"
  "srv/ParamSet.srv"
  "srv/GotoLocal.srv"
  "srv/GotoGlobal.srv"
  "srv/SetMode.srv"
  "srv/VelocityVector.srv"
  "srv/MessageInterval.srv"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()