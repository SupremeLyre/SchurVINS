cmake_minimum_required(VERSION 3.8)
project(svo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${messages_to_build}
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
