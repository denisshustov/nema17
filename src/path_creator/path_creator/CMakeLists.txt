cmake_minimum_required(VERSION 3.0.2)
project(path_creator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  map_contur_msg
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   contur.msg
# )

# Generate services in the 'srv' folder
add_service_files(
  FILES
  way_points_srv.srv
  conturs_srv.srv
  conturs_by_point_srv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs map_contur_msg
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_creator
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs message_runtime map_contur_msg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})