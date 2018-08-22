cmake_minimum_required(VERSION 2.8.3)
project(hearts_top_pov)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib

  pcl_conversions
  pcl_ros

  dynamic_reconfigure
  tf

  cv_bridge
  image_transport
  
  hearts_tools

  hearts_nav_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

catkin_package()

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/lineransac.cpp src/linefinder.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(rgbd_src src/rgbd_src.cpp)
#target_link_libraries(tiago_brl_src ${catkin_LIBRARIES})

#add_executable(top_pov_node src/top_pov_node.cpp
#	           src/lineransac.cpp src/linefinder.cpp )
#target_link_libraries(top_pov_node ${catkin_LIBRARIES})

add_executable(top_pov_node src/top_pov_node.cpp )
target_link_libraries(top_pov_node 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(top_pov_lis src/top_pov_lis.cpp )
target_link_libraries(top_pov_lis 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

#add_executable(top_pov_srv src/top_pov_srv.cpp
#	           src/lineransac.cpp src/linefinder.cpp )
#target_link_libraries(top_pov_srv ${catkin_LIBRARIES})

add_executable(top_pov_srv src/top_pov_srv.cpp )
target_link_libraries(top_pov_srv 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(top_pov_cli src/top_pov_cli.cpp )
target_link_libraries(top_pov_cli 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

### INSTALL ###

#install(TARGETS ${PROJECT_NAME} rgbd_src top_pov
install(TARGETS top_pov_node top_pov_lis top_pov_srv top_pov_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
