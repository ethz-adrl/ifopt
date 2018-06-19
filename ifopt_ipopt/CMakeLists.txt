cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_ipopt)

###########
## Build ##
###########
# The IPOPT solver interface
add_library(${PROJECT_NAME} SHARED src/ipopt_adapter.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${IPOPT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC  ifopt_core
  PRIVATE ${IPOPT_LIB}
)


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
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building IPOPT test executable \"testipopt\"")
  add_executable(testipopt test/ex_test_ipopt.cc)
    
  target_link_libraries(testipopt PRIVATE 
    ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()