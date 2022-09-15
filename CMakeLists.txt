# Copyright (c) 2017, Alexander W. Winkler, ETH Zurich. All rights reserved.
cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ifopt VERSION 2.0.6)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

###########
## Build ##
###########
include(GNUInstallDirs) # for correct library locations across platforms
set(config_package_location "share/${PROJECT_NAME}/cmake") # for .cmake find-scripts installs
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)             # so installed solver libraries link to IFOPT/SNOPT                                          
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)    # so installed solver libraries link to libifopt_core.so  
set(IFOPT_INSTALL_BINDIR ${CMAKE_INSTALL_LIBDIR}/ifopt) # replicate default for catkin installs

IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

enable_testing()
set(LIB_CORE ifopt_core)
add_subdirectory(ifopt_core)

option(BUILD_IPOPT "compile with IPOPT solver" ON)
if (BUILD_IPOPT)
  add_subdirectory(ifopt_ipopt)
endif()

option(BUILD_SNOPT "compile with SNOPT solver" OFF)
if (BUILD_SNOPT)
  add_subdirectory(ifopt_snopt)
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
# install the two files in a place where cmake looks for them
install(
  FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/ifopt-config.cmake"         # self-written
    "${CMAKE_CURRENT_BINARY_DIR}/ifopt-config-version.cmake" # generated
  DESTINATION ${config_package_location}
)
# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

##########################
## Build binary package ##
##########################
if(BUILD_PACKAGE)
  set(DESCRIPTION "An Eigen-based, light-weight C++ Interface to Nonlinear Programming Solvers (Ipopt, Snopt)")
  set(LINUX_DEPENDS "libeigen3-dev")
  if(BUILD_IPOPT)
    list(APPEND LINUX_DEPENDS "coinor-libipopt-dev")
  endif()

  set(CPACK_PACKAGE_VENDOR "ETHZ-ADRL")
  set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_LIST_DIR}/LICENSE)
  set(CPACK_RESOURCE_FILE_README ${CMAKE_CURRENT_LIST_DIR}/README.md)
  if(UNIX)
    set(CPACK_GENERATOR "DEB;TXZ")

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
      set(DEB_ARCH "amd64")
    else()
      set(DEB_ARCH ${CMAKE_SYSTEM_PROCESSOR})
    endif()

    set(CPACK_PACKAGE_FILE_NAME "${ARG_PACKAGE_PREFIX}${PROJECT_NAME}_${DEB_ARCH}_linux_${CMAKE_PROJECT_VERSION}")
    set(CPACK_DEBIAN_PACKAGE_NAME "${ARG_PACKAGE_PREFIX}${PROJECT_NAME}")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${DEB_ARCH})
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER <https://github.com/ethz-adrl/ifopt>)
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION ${DESCRIPTION})
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS=ON)
    string(REPLACE ";" "," CPACK_DEBIAN_PACKAGE_DEPENDS "${LINUX_DEPENDS}")
  include(CPack)
  endif()
endif()
