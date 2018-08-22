cmake_minimum_required(VERSION 2.8.3)
project(hearts_head_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  actionlib
  
  hearts_nav_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
)

catkin_package()

include_directories(SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(head_pose_srv src/head_pose_srv.cpp )
target_link_libraries(head_pose_srv ${catkin_LIBRARIES})

add_executable(single_pose_cli src/single_pose_cli.cpp )
target_link_libraries(single_pose_cli ${catkin_LIBRARIES})

add_executable(loop_pose_cli src/loop_pose_cli.cpp )
target_link_libraries(loop_pose_cli ${catkin_LIBRARIES})

### INSTALL ###

#install(TARGETS ${PROJECT_NAME} rgbd_src top_pov
install(TARGETS head_pose_srv single_pose_cli loop_pose_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
