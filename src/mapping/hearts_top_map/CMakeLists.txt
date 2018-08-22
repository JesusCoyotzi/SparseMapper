cmake_minimum_required(VERSION 2.8.3)
project(hearts_top_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib
  
  hearts_tools

  hearts_nav_msgs
  nav_msgs
  std_msgs
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

#add_executable(top_map_srv src/top_map_srv.cpp
#	           src/lineransac.cpp src/linefinder.cpp )
#target_link_libraries(top_map_srv ${catkin_LIBRARIES})

add_executable(top_map_srv src/top_map_srv.cpp )
target_link_libraries(top_map_srv 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(top_map_cli src/top_map_cli.cpp)
target_link_libraries(top_map_cli 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(top_pov2map_srv src/top_pov2map_srv.cpp )
target_link_libraries(top_pov2map_srv 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

add_executable(top_pov2map_cli src/top_pov2map_cli.cpp)
target_link_libraries(top_pov2map_cli 
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES})

### INSTALL ###

#install(TARGETS ${PROJECT_NAME} rgbd_src top_pov
install(TARGETS top_map_srv top_map_cli top_pov2map_srv top_pov2map_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
