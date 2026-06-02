cmake_minimum_required(VERSION 3.8)
project(fast)

find_package(ament_cmake REQUIRED)

set(HEADERS
  include/fast/corner_9.h
  include/fast/corner_10.h
  include/fast/fast.h
  include/fast/faster_corner_utilities.h
  )

set(SOURCES
  src/fast_9.cpp
  src/fast_10.cpp
  src/fast_10_score.cpp
  src/nonmax_3x3.cpp
  )

IF(DEFINED ENV{ARM_ARCHITECTURE})
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_9_neon.cpp)
ELSE()
  LIST(APPEND SOURCES ${PROJECT_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
ENDIF()

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wno-unused-variable
  -Wno-unused-but-set-variable
  -Wno-unknown-pragmas
  -O3
)
if(DEFINED ENV{ARM_ARCHITECTURE})
  target_compile_options(${PROJECT_NAME} PRIVATE -mfpu=neon -march=armv7-a)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE
    -mmmx -msse -msse2 -msse3 -mssse3 -fomit-frame-pointer)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
ament_package()
