cmake_minimum_required(VERSION 2.8)

PROJECT(test)

include_directories("/usr/local/eigen3")
add_executable(test  main.cpp)
add_executable(map_useage map_useage.cpp)
