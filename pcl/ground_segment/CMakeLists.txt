cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(segment)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ground_segment ground_segment.cpp)
target_link_libraries (ground_segment ${PCL_LIBRARIES})
