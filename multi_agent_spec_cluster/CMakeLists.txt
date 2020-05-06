cmake_minimum_required(VERSION 2.8.3)
project(multi_agent_spec_cluster)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  multi_agent_messages
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  scout_robot_node src/scout_robot.cpp
)

target_link_libraries(
  scout_robot_node ${catkin_LIBRARIES}
)

add_dependencies(
  scout_robot_node ${catkin_EXPORTED_TARGETS}
)

add_executable(
  cluster_robot_node src/cluster_robot.cpp
)

target_link_libraries(
  cluster_robot_node ${catkin_LIBRARIES}
)

