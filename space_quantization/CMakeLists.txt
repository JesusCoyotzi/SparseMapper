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
#message(AUTHOR_WARNING "Los directorios de PCL son:" ${PCL_LIBRARY_DIRS} )

find_package(CUDA REQUIRED)
find_package(catkin REQUIRED cmake_modules)

#This is required if PCL to be compiled as part of the package, it is a bug
set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
# On cuda 9.1 and PCL 1.7.1 erase it and this package won't compile
#Hopefully on melodic and pcl 1.8 it wont matter.

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   tf
   sensor_msgs
   pcl_conversions
   pcl_ros
   visualization_msgs
   sparse_map_msgs
)

 # Declare that this catkin package's runtime dependencies
 catkin_package(
  CATKIN_DEPENDS  std_msgs sensor_msgs geometry_msgs sparse_map_msgs
 )


 include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 )

 #Cuda stuff
 #This is important set to your require compute capability, this was compiled
 #for a Gforce 1050 this compute capability 61
 if(CUDA_FOUND)
 set(
   CUDA_NVCC_FLAGS
   ${CUDA_NVCC_FLAGS};
   -O3 -lineinfo -gencode arch=compute_61,code=sm_61 -std=c++11
   )
   set(CUDA_PROPAGATE_HOST_FLAGS OFF)
   set(CUDA_SEPARABLE_COMPILATION ON)
   set(CUDA_VERBOSE_BUILD OFF)
   cuda_add_library(segmentationLib STATIC src/gpu/operations.cu src/gpu/quantization.cu  src/gpu/cpuQuantization.cu)

   add_executable(segmentation_node src/gpu/space_seg.cpp include/space_seg.h src/seg_node.cpp)
   add_dependencies(segmentation_node segmentationLib ${catkin_EXPORTED_TARGETS})
   target_link_libraries(segmentation_node segmentationLib ${catkin_LIBRARIES})# segmentationLib)
 else()
  add_library(segmentationLib STATIC src/cpu/operations.cpp src/cpu/cpuQuantization.cpp)
  add_executable(segmentation_node src/cpu/space_seg_cpu.cpp include/space_seg_cpu.h src/seg_node.cpp)
  add_dependencies(segmentation_node segmentationLib ${catkin_EXPORTED_TARGETS})
  target_link_libraries(segmentation_node segmentationLib ${catkin_LIBRARIES})
endif()


 # define executable using MyMessage1 etc.
 #add_executable(message_program src/main.cpp)
 #add_dependencies(message_program ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 # define executable not using any messages/services provided by this package


 add_executable(pose_chck src/pose_chck_node.cpp src/pose_chck.cpp)
 target_link_libraries(pose_chck ${catkin_LIBRARIES})

 add_executable(cloud_preprocessor src/preprocessing_node.cpp src/preprocessing.cpp)
 target_link_libraries(cloud_preprocessor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
