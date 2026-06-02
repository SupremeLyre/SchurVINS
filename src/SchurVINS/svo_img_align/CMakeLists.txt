cmake_minimum_required(VERSION 3.8)
project(svo_img_align)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(svo_cmake REQUIRED)
find_package(svo_common REQUIRED)
find_package(svo_direct REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_solver REQUIRED)

include(SvoSetup)

set(HEADERS_CPU
  include/svo/img_align/sparse_img_align_base.h
  include/svo/img_align/sparse_img_align.h
)

set(SOURCES_CPU
  src/sparse_img_align_base.cpp
  src/sparse_img_align.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES_CPU} ${HEADERS_CPU})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  glog::glog
  svo_common::svo_common
  svo_direct::svo_direct
  vikit_common::vikit_common
  vikit_solver::vikit_solver
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
ament_export_dependencies(Eigen3 glog OpenCV svo_common svo_direct vikit_common vikit_solver)
ament_package()
