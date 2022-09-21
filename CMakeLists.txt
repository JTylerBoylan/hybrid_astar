cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/local_planner.cpp
)

