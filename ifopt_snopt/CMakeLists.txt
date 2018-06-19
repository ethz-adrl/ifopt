cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_snopt)

###########
## Build ##
###########
if(${SNOPT76})
 add_library(${PROJECT_NAME} SHARED src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} SHARED src/snopt_adapter.cc)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC 
  include
  ${SNOPT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ifopt_core
  ${SNOPT_LIB1}
  ${SNOPT_LIB2}
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
#  DIRECTORY include/ifopt_snopt/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)


#############
## Testing ##
#############
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building SNOPT test executable \"testsnopt\"")
  add_executable(testsnopt test/ex_test_snopt.cc)
    
  target_link_libraries(testsnopt PRIVATE 
    ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()
