cmake_minimum_required(VERSION 2.8.3)
project(multi_agent_messages)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation 
  std_msgs 
  geometry_msgs
  nav_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Communication.msg
)

## Generate services in the 'srv' folder
#add_service_files( FILES
#  Service.srv
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)