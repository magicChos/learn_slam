cmake_minimum_required( VERSION 2.8 )
project( bundle_adjustment )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set(OpenCV_DIR /usr/local/Cellar/opencv/4.2.0/lib/cmake/opencv4)
FIND_PACKAGE( OpenCV )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories("/usr/local/eigen3")
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
)


add_executable( pose_estimation_3d2d main.cpp )
target_link_libraries( pose_estimation_3d2d
   ${OpenCV_LIBS}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
)

