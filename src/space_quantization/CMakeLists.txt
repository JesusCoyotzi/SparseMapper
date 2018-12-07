cmake_minimum_required(VERSION 2.8.3)
project(space_quantization)
#set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
#//can't compile
#Ok this is ridiculous, PCL 1.7 is builts with c++ 0x
#And thus i can't use  c++ 11.
#Ros kinetic defaults to PCL 1.7 and i am unable to change
#to 1.8 as it is hardcoded to use 1.7 in most prebuilt binaries
# Get the information about this package's buildtime dependencies
# and on runtime.
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Esto ya es ocio
message(AUTHOR_WARNING "Los directions de PCL son:" ${PCL_LIBRARY_DIRS} )

find_package(CUDA REQUIRED)
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   tf
   sensor_msgs
   pcl_conversions
   pcl_ros
   visualization_msgs
   message_generation
   )



set(
 CUDA_NVCC_FLAGS
 ${CUDA_NVCC_FLAGS};
 -O3 -lineinfo -gencode arch=compute_61,code=sm_61 -std=c++11
 )
 SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

 SET(CUDA_SEPARABLE_COMPILATION ON)

set(CUDA_VERBOSE_BUILD OFF)

 # Declare the message files to be built
add_message_files(FILES
   quantizedSpace.msg
   codebook.msg
)

 # Declare the service files to be built
 #add_service_files(FILES
#   MyService.srv
# )

 # Actually generate the language-specific message and service files

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   )

 # Declare that this catkin package's runtime dependencies
 catkin_package(
  CATKIN_DEPENDS  std_msgs sensor_msgs geometry_msgs
  message_runtime
  DEPENDS Eigen
 )


 include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
 )

 #Cuda stuff
cuda_add_library(segmentationLib STATIC src/operations.cu src/quantization.cu  src/cpuQuantization.cu)

 # define executable using MyMessage1 etc.
 #add_executable(message_program src/main.cpp)
 #add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 # define executable not using any messages/services provided by this package
 add_executable(segmentation_node src/space_seg.cpp include/space_seg.h src/seg_node.cpp)
 add_dependencies(segmentation_node segmentationLib)
 target_link_libraries(segmentation_node segmentationLib ${catkin_LIBRARIES})# segmentationLib)

 add_executable(pose_chck src/pose_chck_node.cpp src/pose_chck.cpp)
 target_link_libraries(pose_chck ${catkin_LIBRARIES})

 add_executable(cloud_preprocessor src/preprocessing_node.cpp src/preprocessing.cpp)
 target_link_libraries(cloud_preprocessor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(adjacency_graph_node src/adjacency_graph_node.cpp src/adjacency_graph.cpp)
 #target_link_libraries(adjacency_graph_node ${catkin_LIBRARIES})


#add_executable(testColor src/space_seg.cpp include/space_seg.h test/colortest.cpp)
# target_link_libraries(testColor segmentationLib ${catkin_LIBRARIES})
