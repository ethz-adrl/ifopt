cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(Eigen3 REQUIRED)
find_package(catkin QUIET) # optional: allows to be found in catkin workspace  
find_package(GTest  QUIET) # optional: enables/disables testing


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
# This can then be used to "find_package(ifopt)" if built with catkin
if (catkin_FOUND)
catkin_package(
  INCLUDE_DIRS ifopt_core/include
               ifopt_ipopt/include ${IPOPT_INCLUDE_DIRS}
               ifopt_snopt/include ${SNOPT_INCLUDE_DIRS}
               ${EIGEN3_INCLUDE_DIR}
                                         
  LIBRARIES    ifopt_core
               ${IFOPT_IPOPT_LIB} ${IPOPT_LIB}
               ${IFOPT_SNOPT_LIB} ${SNOPT_LIB1} ${SNOPT_LIB2}
                
  DEPENDS      eigen  
)
endif()


###########
## Build ##
###########
# Default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_subdirectory(ifopt_core)

if (IFOPT_IPOPT_LIB)
  add_subdirectory(ifopt_ipopt)
endif()

if (IFOPT_SNOPT_LIB)
  add_subdirectory(ifopt_snopt)
endif()




