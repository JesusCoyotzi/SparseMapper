cmake_minimum_required(VERSION 3.1)
project( occGridProc )
set (CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( occGridProc occGridProc.cpp )
target_link_libraries( occGridProc ${OpenCV_LIBS} )
