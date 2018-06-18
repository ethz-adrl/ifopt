cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_ipopt)

#find_package(catkin REQUIRED)


################
## Find IPOPT ##
################
# adapt this to point to your installed IPOPT folder
set(IPOPT_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.8")
if(IS_DIRECTORY ${IPOPT_DIR}/build/include/coin)
  set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/build/include/coin)
  set(IPOPT_LIBRARIES    ${IPOPT_DIR}/build/lib/libipopt.so)
else()
  message(WARNING "IPOPT directory \"" ${IPOPT_DIR} "\" NOT found " 
                  "-> Not compiling ifopt_ipopt. \n" 
                  "Specify path to your installed IPOPT installation here.")
  return()
endif()


#################################
## Generate cmake config files ##
#################################
# These can then be used to "find_package(ifopt_ipopt)"
#catkin_package(
#  INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS} ${ifopt_core_INCLUDE_DIRS}
#  LIBRARIES    ${PROJECT_NAME}
#)


###########
## Build ##
###########
# The IPOPT solver interface
add_library(${PROJECT_NAME} STATIC src/ipopt_adapter.cc)
target_include_directories(${PROJECT_NAME} PUBLIC 
  include
  ${IPOPT_INCLUDE_DIRS}
  ${ifopt_core_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ifopt_core
  ${IPOPT_LIBRARIES}
)

#target_link_libraries(${PROJECT_NAME} PUBLIC
#  ${ifopt_core_LIBRARIES}
#  ${IPOPT_LIBRARIES}
#)


#############
## Install ##
#############
# For this to work, first make sure IPOPT headers and library are installed
# somewhere in the default search paths.
# Mark library for installation
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark header files for installation
#install(
#  DIRECTORY include/ifopt_ipopt/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)


#############
## Testing ##
#############
find_package(GTest QUIET)
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building IPOPT test executable \"testipopt\"")
  add_executable(testipopt test/ex_test_ipopt.cc)
    
  target_link_libraries(testipopt PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()