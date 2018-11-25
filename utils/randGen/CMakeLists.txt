cmake_minimum_required(VERSION 2.8.3)
project(uniformCloud)

find_package(CUDA REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(uniformCloud uniformCloud.cpp)
target_link_libraries(uniformCloud  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  )
