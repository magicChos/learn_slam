cmake_minimum_required(VERSION 3.18)
project(open3d_test)

set(CMAKE_CXX_STANDARD 11)

find_package( Open3D  REQUIRED)
find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

message(${Open3D_INCLUDE_DIRS})
include_directories(${Open3D_INCLUDE_DIRS} /usr/local/include /usr/local/include/open3d/3rdparty )
link_directories(${Open3D_LIBRARY_DIRS})

add_executable(TestVisualizer main.cpp)
add_executable(voxelization Voxelization.cpp)
target_link_libraries(TestVisualizer ${Open3D_LIBRARIES} pthread)
target_link_libraries(voxelization ${Open3D_LIBRARIES})

target_include_directories(TestVisualizer PUBLIC ${Open3D_INCLUDE_DIRS})
target_include_directories(voxelization PUBLIC ${Open3D_INCLUDE_DIRS})
