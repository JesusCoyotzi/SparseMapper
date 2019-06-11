cmake_minimum_required(VERSION 2.8.3)
project(sparse_map_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sparse_map_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE( Boost  REQUIRED COMPONENTS system filesystem )
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sparse_map_utils
  CATKIN_DEPENDS geometry_msgs pcl roscpp rospy sensor_msgs std_msgs tf sparse_map_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cloudSimulation_node src/cloudSimulation.cpp  src/cloudSimulation_node.cpp)
add_dependencies(cloudSimulation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloudSimulation_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
