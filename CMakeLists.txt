cmake_minimum_required(VERSION 2.8.3)
project(ifopt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++14")
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)



## check for which solver the interface should be compiled. This is determined
## from whether the environmental variable is set in the ~/.bashrc, e.g.
## export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.4
## export SNOPT_DIR=/home/path/to/snopt/snopt_lib
if(NOT DEFINED ENV{IPOPT_DIR})
  message(STATUS "IPOPT_DIR not set. Not compiling for IPOPT")
else()
  message(STATUS "IPOPT_DIR set. Compiling with IPOPT.")
  set(IPOPT_INCLUDE_DIRS $ENV{IPOPT_DIR}/build/include/coin)
  set(IPOPT_LIBRARIES    $ENV{IPOPT_DIR}/build/lib/libipopt.so)
  set(IPOPT_ADAPTER_LIB ifopt_ipopt)
  option(IPOPT_FOUND "Build IPOPT." ON)
endif()

if(NOT DEFINED ENV{SNOPT_DIR})
  message(STATUS "SNOPT_DIR not set. Not compiling for SNOPT")
else()
  message(STATUS "SNOPT_DIR set. Compiling with SNOPT.")
  set(SNOPT_INCLUDE_DIRS $ENV{SNOPT_DIR}/include)
  set(SNOPT_LIBRARIES $ENV{SNOPT_DIR}/lib/libsnopt7_cpp.so 
                      $ENV{SNOPT_DIR}/lib/libsnopt7.so)
  set(SNOPT_ADAPTER_LIB ifopt_snopt)
  option(SNOPT_FOUND "Build SNOPT." ON)
endif()





find_package(catkin QUIET)
if(catkin_FOUND)
##################################
# catkin specific configuration ##
##################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
               ${EIGEN3_INCLUDE_DIR} 
               ${IPOPT_INCLUDE_DIRS} 
               ${SNOPT_INCLUDE_DIRS}
  LIBRARIES    ${PROJECT_NAME}
               ${IPOPT_ADAPTER_LIB}
               ${SNOPT_ADAPTER_LIB}
               ${IPOPT_LIBRARIES} 
               ${SNOPT_LIBRARIES}
)
endif()



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
  ${SNOPT_INCLUDE_DIRS}
)

# The solver independent part of the library
add_library(${PROJECT_NAME}
  src/problem.cc
  src/composite.cc
  src/leaves.cc
  src/linear_constraint.cc
  src/soft_constraint.cc
)

# compile adapter for IPOPT 
if(IPOPT_FOUND)
  add_library(${IPOPT_ADAPTER_LIB} src/ipopt_adapter.cc)
endif()

# compile adapter for SNOPT 
if(SNOPT_FOUND)
  add_library(${SNOPT_ADAPTER_LIB} src/snopt_adapter.cc)
endif()


#############
## Testing ##
#############
enable_testing()
add_subdirectory(test)

