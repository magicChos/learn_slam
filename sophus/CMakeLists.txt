cmake_minimum_required( VERSION 2.8 )
project( useSophus )

# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( "/usr/local/eigen3")

add_executable( useSophus main.cpp )
target_link_libraries( useSophus "/usr/local/lib/libSophus.dylib" )