cmake_minimum_required(VERSION 2.8.3)
project(ifopt_ipopt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ifopt
)


## environmental variable is set in the ~/.bashrc, e.g.
## export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.4
message(STATUS "IPOPT_DIR not set. Not compiling for IPOPT")

message(STATUS "IPOPT_DIR set to " $ENV{IPOPT_DIR})
set(IPOPT_INCLUDE_DIRS $ENV{IPOPT_DIR}/build/include/coin)
set(IPOPT_LIBRARIES    $ENV{IPOPT_DIR}/build/lib/libipopt.so)



catkin_package(
  INCLUDE_DIRS   include
                 ${IPOPT_INCLUDE_DIRS}
  LIBRARIES      ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${IPOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# The IPOPT solver interface
add_library(${PROJECT_NAME}
  src/ipopt_adapter.cc
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
)


#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_ipopt.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
  )
endif()

