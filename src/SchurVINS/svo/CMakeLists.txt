cmake_minimum_required(VERSION 3.8)
project(svo)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(opengv REQUIRED)
find_package(rpg_common REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_direct REQUIRED)
find_package(svo_img_align REQUIRED)
find_package(svo_tracker REQUIRED)
find_package(svo_vio_common REQUIRED)
find_package(vikit_cameras REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(vikit_solver REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

include(SvoSetup)

set(SVO_SOURCEFILES
  src/frame_handler_mono.cpp
  src/frame_handler_stereo.cpp
  src/frame_handler_array.cpp
  src/frame_handler_base.cpp
  src/map.cpp
  src/pose_optimizer.cpp
  src/schur_vins.cpp
  src/initialization.cpp
  src/reprojector.cpp
  src/imu_handler.cpp
  src/stereo_triangulation.cpp
  src/io.cpp
)

set(SVO_HEADERFILES
  include/svo/abstract_bundle_adjustment.h
  include/svo/frame_handler_base.h
  include/svo/frame_handler_mono.h
  include/svo/frame_handler_stereo.h
  include/svo/frame_handler_array.h
  include/svo/global.h
  include/svo/imu_handler.h
  include/svo/initialization.h
  include/svo/map.h
  include/svo/pose_optimizer.h
  include/svo/schur_vins.h
  include/svo/reprojector.h
  include/svo/stereo_triangulation.h
  include/svo/io.h
  include/svo/svo.h
)

add_library(${PROJECT_NAME} SHARED ${SVO_SOURCEFILES} ${SVO_HEADERFILES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  glog::glog
  opengv::opengv
  rpg_common::rpg_common
  svo_common::svo_common
  svo_direct::svo_direct
  svo_img_align::svo_img_align
  svo_tracker::svo_tracker
  svo_vio_common::svo_vio_common
  vikit_cameras::vikit_cameras
  vikit_common::vikit_common
  vikit_ros::vikit_ros
  vikit_solver::vikit_solver
  yaml-cpp::yaml-cpp
  ${OpenCV_LIBRARIES}
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
  Eigen3
  glog
  OpenCV
  opengv
  rpg_common
  svo_common
  svo_direct
  svo_img_align
  svo_tracker
  svo_vio_common
  vikit_cameras
  vikit_common
  vikit_ros
  vikit_solver
  yaml-cpp
)
ament_package()
