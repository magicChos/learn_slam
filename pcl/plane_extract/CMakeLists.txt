cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(livox_pcd)

#set(CMAKE_BUILD_TYPE  Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()


find_package(PCL)
find_package( OpenMP )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(plane_extraction plane_extraction.cpp)

if(OpenMP_CXX_FOUND)
    target_link_libraries(plane_extraction PUBLIC  pthread OpenMP::OpenMP_CXX ${PCL_LIBRARIES})
else()
    target_link_libraries (plane_extraction ${PCL_LIBRARIES})
endif()