cmake_minimum_required(VERSION 2.8.3)
project(sparse_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  rospy
  visualization_msgs
  sparse_map_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sparse_mappper
#  CATKIN_DEPENDS geometry_msgs std_msgs visualization_msgs sparse_map_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sparse_mapper_node.cpp src/sparse_mapper.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
