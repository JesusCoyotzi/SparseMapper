cmake_minimum_required(VERSION 2.8.3)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(cloudConverter)
#set (CMAKE_CXX_STANDARD 11) for cmake 3.1 and newr

#set(CMAKE_CXX_STANDARD 11) # C++11...
#set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
#set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
find_package(PCL 1.8 REQUIRED)# COMPONENTS common io visualization)
#if (NOT PCL)
#  message(FATAL_ERROR "PCL not found!!!!" )
#endif()
#list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_compile_options(-std=c++11)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
FIND_PACKAGE( Boost  REQUIRED COMPONENTS system filesystem )

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(pcdConverter pcdConverter.cpp)
target_link_libraries(pcdConverter ${PCL_LIBRARIES})# ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES}  ${Boost_LIBRARIES})
