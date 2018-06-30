# Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ifopt VERSION 2.0.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)

# predefine library names to export with catkin-package 
set(LIB_CORE ifopt_core)
set(ifopt_LIBRARIES ${LIB_CORE})
set(ifopt_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
list(APPEND ifopt_INCLUDE_DIRS ifopt_core/include)

option(BUILD_IPOPT "compile with IPOPT solver" ON)
if (BUILD_IPOPT)
  set(LIB_IPOPT ifopt_ipopt)
  list(APPEND ifopt_INCLUDE_DIRS ifopt_ipopt/include)
  list(APPEND ifopt_LIBRARIES ${LIB_IPOPT})  
endif()

option(BUILD_SNOPT "compile with SNOPT solver" OFF)
if (BUILD_SNOPT)
  set(LIB_SNOPT ifopt_snopt)
  list(APPEND ifopt_INCLUDE_DIRS ifopt_snopt/include)
  list(APPEND ifopt_LIBRARIES ${LIB_SNOPT})
endif()



##############################################
## catkin specific configuration (optional) ##
##############################################
if (CATKIN_DEVEL_PREFIX) # set only when building with catkin, not pure cmake
find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS ${ifopt_INCLUDE_DIRS}     
  LIBRARIES    ${ifopt_LIBRARIES}
)
endif()


###########
## Build ##
###########
include(GNUInstallDirs) # for correct library locations across platforms
set(config_package_location ${CMAKE_INSTALL_PREFIX}/share/ifopt/cmake) # for .cmake find-scripts installs
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)          # so installed solver libraries link to IFOPT/SNOPT                                          
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib) # so installed solver libraries link to libifopt_core.so  

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

enable_testing()
add_subdirectory(${LIB_CORE})

if (BUILD_IPOPT)
  add_subdirectory(${LIB_IPOPT})
endif()

if (BUILD_SNOPT)
  add_subdirectory(${LIB_SNOPT})
endif()


##################################################
## Install find scripts for find_package(ifopt) ##
##################################################
# generate the file IfoptConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ifopt-config-version.cmake"
  VERSION ${ifopt_VERSION}
  COMPATIBILITY SameMajorVersion
)

# install the two files in a place where cmake looks for them so can call
# "find_package(ifopt).
# see also: https://cmake.org/cmake/help/v3.0/manual/cmake-packages.7.html
install(
  FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/ifopt-config.cmake"   # self-written
    "${CMAKE_CURRENT_BINARY_DIR}/ifopt-config-version.cmake" # generated
  DESTINATION ${config_package_location}
)
