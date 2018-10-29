cmake_minimum_required(VERSION 2.8.3)
project(quantization_utilities)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quantization_utilities
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
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
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/quantization_utilities.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tf_saver_node src/tf_saver_node.cpp src/tf_saver.cpp)
target_link_libraries(tf_saver_node
   ${catkin_LIBRARIES}
)

add_executable(tf_reader_node src/tf_reader_node.cpp src/tf_reader.cpp)
target_link_libraries(tf_reader_node
   ${catkin_LIBRARIES}
)
