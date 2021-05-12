cmake_minimum_required(VERSION 3.0.2)
project(path_creator_goal_mover)
find_package(catkin REQUIRED COMPONENTS
  map_contur_msg
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)