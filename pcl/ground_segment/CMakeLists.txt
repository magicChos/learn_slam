cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(segment)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories( . )
set(SRC plane_fitting.cpp ground_segment.cpp)

add_executable (ground_segment ${SRC})
target_link_libraries (ground_segment ${PCL_LIBRARIES})
