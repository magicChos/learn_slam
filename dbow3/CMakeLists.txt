cmake_minimum_required(VERSION 2.8)

project(dbow3)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR /usr/local/Cellar/opencv/4.2.0/lib/cmake/opencv4)
FIND_PACKAGE( OpenCV )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
include_directories( "/usr/local/include" )
set (DBoW3_LIBS "libDBoW3.dylib")
set (StlPlus_LIBS "libopenMVG_stlplus.a")

# 自定义的文件操作库
set (SelfFileOperation_LIBS "libSelfFileOperation.dylib")

add_executable(main "main.cpp")
target_link_libraries(main LINK_PUBLIC ${OpenCV_LIBS} ${DBoW3_LIBS} ${StlPlus_LIBS} ${SelfFileOperation_LIBS})

add_executable(use_dbow "use_dbow.cpp")
target_link_libraries(use_dbow LINK_PUBLIC ${OpenCV_LIBS} ${DBoW3_LIBS} ${StlPlus_LIBS} ${SelfFileOperation_LIBS})