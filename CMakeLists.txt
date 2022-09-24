cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  roscpp
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hybrid_astar
  CATKIN_DEPENDS grid_map_ros roscpp nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/planner.cpp
)

add_executable(planner src/planning_node.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${PROJECT_NAME})

catkin_install_python(PROGRAMS scripts/mat2gridmap.py scripts/odometry.py scripts/goal_point.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


