cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(livox_pcd)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_extraction plane_extraction.cpp)
target_link_libraries (plane_extraction ${PCL_LIBRARIES})
