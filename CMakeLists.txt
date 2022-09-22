cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_planner
  CATKIN_DEPENDS grid_map_ros roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/local_planner.cpp
)

catkin_install_python(PROGRAMS scripts/mat2gridmap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

