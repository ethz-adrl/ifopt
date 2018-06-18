# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

enable_testing()
add_subdirectory(ifopt_core)
add_subdirectory(ifopt_ipopt)
