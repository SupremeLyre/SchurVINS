cmake_minimum_required(VERSION 3.8)
project(vikit_ros)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(minkindr REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

include(SvoSetup)

set(HEADERS
  include/vikit/img_type_conversion.h
  include/vikit/output_helper.h
  include/vikit/params_helper.h
)

set(SOURCES
  src/output_helper.cpp
  src/img_type_conversion.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  glog::glog
  minkindr::minkindr
  rclcpp::rclcpp
  vikit_common::vikit_common
  yaml-cpp::yaml-cpp
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  cv_bridge
  geometry_msgs
  sensor_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  cv_bridge
  Eigen3
  geometry_msgs
  glog
  minkindr
  OpenCV
  rclcpp
  sensor_msgs
  tf2_ros
  vikit_common
  visualization_msgs
  yaml-cpp
)
ament_package()
