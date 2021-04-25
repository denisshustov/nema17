cmake_minimum_required(VERSION 3.0.2)
project(map_contur_msg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  map_contur_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_contur_msg
  CATKIN_DEPENDS geometry_msgs rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

