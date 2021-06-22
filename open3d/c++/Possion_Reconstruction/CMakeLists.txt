cmake_minimum_required(VERSION 3.18)
project(Possion_Reconstruction)

set(CMAKE_CXX_STANDARD 11)

find_package( Open3D  REQUIRED)
find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

message(${Open3D_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS} /usr/local/include /usr/local/include/open3d/3rdparty )
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(possion_reconstruction main.cpp)
target_link_libraries(possion_reconstruction ${Open3D_LIBRARIES} pthread)
target_include_directories(possion_reconstruction PUBLIC ${Open3D_INCLUDE_DIRS})
