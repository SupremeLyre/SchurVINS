cmake_minimum_required(VERSION 3.8)
project(svo_ros)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(rpg_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(svo REQUIRED)
find_package(svo_ceres_backend REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_msgs REQUIRED)
find_package(svo_tracker REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(${OpenCV_VERSION} STREQUAL "3.2.0")
  message(STATUS "OpenCV version ${OpenCV_VERSION}")
else()
  message(WARNING "OpenCV version ${OpenCV_VERSION}, OpenCV version must be 3.2.0")
endif()

include(SvoSetup)

set(HEADERS
  include/svo_ros/svo_interface.h
  include/svo_ros/svo_factory.h
  include/svo_ros/visualizer.h
  include/svo_ros/csv_dataset_reader.h
  include/svo_ros/ceres_backend_factory.h
  include/svo_ros/svo_node_base.h
)

set(SOURCES
  src/ceres_backend_factory.cpp
  src/svo_interface.cpp
  src/svo_factory.cpp
  src/svo_node_base.cpp
  src/visualizer.cpp
  src/csv_dataset_reader.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  gflags
  glog::glog
  rclcpp::rclcpp
  rpg_common::rpg_common
  svo::svo
  svo_ceres_backend::svo_ceres_backend
  svo_common::svo_common
  svo_tracker::svo_tracker
  vikit_common::vikit_common
  vikit_ros::vikit_ros
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ament_index_cpp
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  svo_msgs
  tf2_ros
  visualization_msgs
)

add_executable(svo_node src/svo_node.cpp)
target_compile_features(svo_node PUBLIC cxx_std_17)
target_link_libraries(svo_node PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} svo_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)
install(FILES
  rviz_config.rviz
  rviz_config_gm.rviz
  rviz_config_vio.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  cv_bridge
  Eigen3
  gflags
  glog
  geometry_msgs
  message_filters
  nav_msgs
  OpenCV
  pcl_conversions
  PCL
  rclcpp
  rpg_common
  sensor_msgs
  std_msgs
  svo
  svo_ceres_backend
  svo_common
  svo_msgs
  svo_tracker
  tf2_ros
  vikit_common
  vikit_ros
  visualization_msgs
)
ament_package()
