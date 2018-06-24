# Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ifopt VERSION 1.1.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
# optional packages
find_package(ipopt  QUIET)
find_package(snopt  QUIET)



set(LIB_CORE ifopt_core)
##############################################
## catkin specific configuration (optional) ##
##############################################
if (CATKIN_DEVEL_PREFIX) # set only when building with catkin, not pure cmake
find_package(catkin REQUIRED)
catkin_package(
  INCLUDE_DIRS src/ifopt_core/include
               src/ifopt_ipopt/include 
               src/ifopt_snopt/include
               ${EIGEN3_INCLUDE_DIR}
                                         
  LIBRARIES    ${LIB_CORE}
               ${LIB_IPOPT}
               ${LIB_SNOPT}
)
endif()


###########
## Build ##
###########
include(GNUInstallDirs) # for correcnt libraries locations across platforms
set(config_package_location share/ifopt/cmake)       # for .cmake find-scripts installs
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)          # so installed solver libraries link to IFOPT/SNOPT                                          
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib) # so installed solver libraries link to libifopt_core.so  

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

enable_testing()
add_subdirectory(src/ifopt_core)

if (LIB_IPOPT)
  add_subdirectory(src/ifopt_ipopt)
endif()

if (LIB_SNOPT)
  add_subdirectory(src/ifopt_snopt)
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
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ifopt-config.cmake"   # self-written
    "${CMAKE_CURRENT_BINARY_DIR}/ifopt-config-version.cmake" # generated
  DESTINATION ${config_package_location}
)
