cmake_minimum_required(VERSION 3.8)
project(svo_cmake)

find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake/Modules
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(CONFIG_EXTRAS cmake/svo_cmake-extras.cmake)
