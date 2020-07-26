cmake_minimum_required(VERSION 2.8)

project(use_LK)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR /usr/local/Cellar/opencv/4.2.0/lib/cmake/opencv4)
FIND_PACKAGE( OpenCV )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(use_LK LINK_PUBLIC ${OpenCV_LIBS})
