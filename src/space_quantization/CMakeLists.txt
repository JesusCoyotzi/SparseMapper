cmake_minimum_required(VERSION 2.8.3)
project(space_quantization)
#set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}") //can't compile
# Get the information about this package's buildtime dependencies
find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   tf
   sensor_msgs
   pcl_conversions
   pcl_ros
   visualization_msgs
   )

set(
 CUDA_NVCC_FLAGS
 ${CUDA_NVCC_FLAGS};
 -O3 -lineinfo -gencode arch=compute_61,code=sm_61
 )

set(CUDA_VERBOSE_BUILD ON)

 # Declare the message files to be built
# add_message_files(FILES
#   MyMessage1.msg
#   MyMessage2.msg
 #)

 # Declare the service files to be built
 #add_service_files(FILES
#   MyService.srv
# )

 # Actually generate the language-specific message and service files
 #generate_messages(DEPENDENCIES std_msgs sensor_msgs)

 # Declare that this catkin package's runtime dependencies
 catkin_package(
  #CATKIN_DEPENDS  std_msgs sensor_msgs
 )

 include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 )

 #Cuda stuff
cuda_add_library(segmentationLib STATIC src/quantization.cu)

 # define executable using MyMessage1 etc.
 #add_executable(message_program src/main.cpp)
 #add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 # define executable not using any messages/services provided by this package
 add_executable(segmentation_node src/space_seg.cpp include/space_seg.h src/seg_node.cpp)
 add_dependencies(segmentation_node segmentationLib)
 target_link_libraries(segmentation_node segmentationLib ${catkin_LIBRARIES})# segmentationLib)

add_executable(testColor test/testColor.cpp include/space_seg.h src/seg_node.cpp)
