cmake_minimum_required(VERSION 2.8.3)
project(hearts_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib

  pcl_conversions
  pcl_ros

  dynamic_reconfigure
  tf

  cv_bridge
  image_transport

  hearts_nav_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hearts_tools
  CATKIN_DEPENDS roscpp actionlib pcl_conversions pcl_ros dynamic_reconfigure cv_bridge image_transport 
                 hearts_nav_msgs sensor_msgs control_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(hearts_tools
  src/linefinder.cpp
  src/lineransac.cpp
  src/transformations.cpp
)

add_dependencies(hearts_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hearts_tools 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})
