cmake_minimum_required(VERSION 3.22)
project(lidar_merge)

set(CMAKE_CXX_STANDARD 17)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_executable(merge_point_node
  src/merge.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

