cmake_minimum_required(VERSION 2.8)

PROJECT(test)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
add_executable(main main.cpp)
add_executable(test  test.cpp)
add_executable(map_useage map_useage.cpp)
# add_executable(test_exsample test_exsample.cpp)
add_executable(fitting_plane fitting_plane.cpp)
add_executable(QR QR.cpp)
