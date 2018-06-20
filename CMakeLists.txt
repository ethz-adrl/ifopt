cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt VERSION 1.2)


set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)


################
## Find IPOPT ##
################
# adapt this to point to your installed IPOPT folder
set(IPOPT_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.8")
if(IS_DIRECTORY ${IPOPT_DIR}/build/include/coin)
  set(IFOPT_IPOPT_LIB ifopt_ipopt)
  set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/build/include/coin)
  find_library(IPOPT_LIB ipopt ${IPOPT_DIR}/build/lib)
else()
  message(WARNING "IPOPT directory \"" ${IPOPT_DIR} "\" NOT found " 
                  "-> Not compiling ifopt_ipopt. \n" 
                  "Specify path to your installed IPOPT installation here.")
endif()


################
## Find SNOPT ##
################
# adapt this to point to your installed IPOPT folder
set(SNOPT_DIR "/home/winklera/3rd_party_software/snopt_lib")
set(SNOPT76 FALSE) # set to TRUE if using Snopt 7.6 or greater
if(IS_DIRECTORY ${SNOPT_DIR}/include)
  set(IFOPT_SNOPT_LIB ifopt_snopt)
  set(SNOPT_INCLUDE_DIRS ${SNOPT_DIR}/include)
  find_library(SNOPT_LIB1 snopt7_cpp ${SNOPT_DIR}/lib)
  find_library(SNOPT_LIB2 snopt7 ${SNOPT_DIR}/lib)
  
  if(${SNOPT76})
    message("SNOPT version >= SNOPT 7.6 detected")
  endif()
else()
  message(WARNING "SNOPT directory \"" ${SNOPT_DIR} "\" NOT found "
                  "-> Not compiling ifopt_snopt.\n" 
                  "Specify path to your installed SNOPT installation here.")
endif()


##############################################
## catkin specific configuration (optional) ##
##############################################
# This can also used to generate scripts for "find_package(ifopt)" 
find_package(catkin QUIET) # optional: incorporate into catkin workspace
if (catkin_FOUND)
catkin_package(
  INCLUDE_DIRS src/ifopt_core/include
               src/ifopt_ipopt/include ${IPOPT_INCLUDE_DIRS}  # to find IPOPT
               src/ifopt_snopt/include ${SNOPT_INCLUDE_DIRS}  # to find SNOPT
               ${EIGEN3_INCLUDE_DIR}
                                         
  LIBRARIES    ifopt_core
               ${IFOPT_IPOPT_LIB} ${IPOPT_LIB}                # to use IPOPT
               ${IFOPT_SNOPT_LIB} ${SNOPT_LIB1} ${SNOPT_LIB2} # to use SNOPT
                
  DEPENDS      eigen
)
endif()


###########
## Build ##
###########
IF(NOT CMAKE_BUILD_TYPE MATCHES Release)
    message(STATUS "CMAKE_BUILD_TYPE not set to Release -> impacts performance")
endif()

# To install libraries into correct locations on all platforms.
include(GNUInstallDirs)
# where all .cmake find scripts will be installed
set(config_package_location lib/cmake/ifopt)

add_subdirectory(src/ifopt_core)

if (IFOPT_IPOPT_LIB)
  add_subdirectory(src/ifopt_ipopt)
endif()

if (IFOPT_SNOPT_LIB)
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
