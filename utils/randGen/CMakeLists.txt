cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(rndCloud)
#set (CMAKE_CXX_STANDARD 11) for cmake 3.1 and newr

#set(CMAKE_CXX_STANDARD 11) # C++11...
#set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
#set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
if (NOT PCL)
  message(FATAL_ERROR "PCL not found!!!!" )
endif()

add_compile_options(-std=c++11)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(uniformCloud uniformCloud.cpp)
target_link_libraries(uniformCloud  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  )

add_executable(gaussianCloud gaussianCloud.cpp)
target_link_libraries(gaussianCloud  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}  )
