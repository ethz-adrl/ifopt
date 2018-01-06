cmake_minimum_required(VERSION 2.8.3)
project(ifopt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wfatal-errors -std=c++11")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build: Debug Release" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(Eigen3 REQUIRED)


## check for which solver the interface should be compiled. This is determined
## from whether the environmental variable is set in the ~/.bashrc, e.g.
## export IPOPT_DIR=/home/path/to/ipopt/Ipopt-3.12.4
## export SNOPT_DIR=/home/path/to/snopt/snopt_lib
## for compatibility with SNOPT 7.6. please make sure to set the variable
## export SNOPT76 = true
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
  if($ENV{SNOPT76})
    message(STATUS "SNOPT76 version detected")
  endif()
  set(SNOPT_INCLUDE_DIRS $ENV{SNOPT_DIR}/include)
  find_library(SNOPT_LIB1 snopt7_cpp $ENV{SNOPT_DIR}/lib)
  find_library(SNOPT_LIB2 snopt7 $ENV{SNOPT_DIR}/lib)
  set(SNOPT_LIBRARIES ${SNOPT_LIB1} ${SNOPT_LIB2})
  set(SNOPT_ADAPTER_LIB ifopt_snopt)
  option(SNOPT_FOUND "Build SNOPT." ON)
endif()



# In case this package is build/used with catkin, this enables other packages
# to find the headers and libraries in this package.
find_package(catkin QUIET)
if(catkin_FOUND)
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
)

# compile adapter for IPOPT 
if(IPOPT_FOUND)
  add_library(${IPOPT_ADAPTER_LIB} src/ipopt_adapter.cc)
endif()

# compile adapter for SNOPT 
if(SNOPT_FOUND)
 if($ENV{SNOPT76})
  add_library(${SNOPT_ADAPTER_LIB} src/snopt76_adapter.cc)
 else()
  add_library(${SNOPT_ADAPTER_LIB} src/snopt_adapter.cc)
 endif()
endif()

#############
## Testing ##
#############
enable_testing()
add_subdirectory(test)

