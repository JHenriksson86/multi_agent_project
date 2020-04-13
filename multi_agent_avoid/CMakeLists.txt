cmake_minimum_required(VERSION 2.8.3)
project(multi_agent_avoid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_node src/main.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node ${catkin_LIBRARIES}
)
