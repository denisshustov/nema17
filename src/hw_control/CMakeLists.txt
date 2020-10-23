cmake_minimum_required(VERSION 3.0.2)
project(hw_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hw_control.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C)