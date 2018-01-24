cmake_minimum_required(VERSION 2.8.3)
project(ifopt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++11")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build: Debug Release" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(Eigen3 REQUIRED)


# In case this package is build/used with catkin, this enables other packages
# to find the headers and libraries in this package.
find_package(catkin QUIET)
if(catkin_FOUND)
catkin_package(
  INCLUDE_DIRS include
               ${EIGEN3_INCLUDE_DIR} 
  LIBRARIES    ${PROJECT_NAME}
)
endif()


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# The solver independent part of the library
add_library(${PROJECT_NAME}
  src/problem.cc
  src/composite.cc
  src/leaves.cc
)


#############
## Testing ##
#############
enable_testing()
add_subdirectory(test)

