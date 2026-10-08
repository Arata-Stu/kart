cmake_minimum_required(VERSION 3.16)
project(kart_interfaces)
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ReferenceLine.msg"
  "msg/BagRequest.msg" "msg/BagStatus.msg"
  "msg/ControlTrim.msg" "msg/ControlCommand.msg" "msg/OperationModeRequest.msg" "msg/OperationModeState.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_auto_package()
