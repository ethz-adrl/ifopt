cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_core)

###########
## Build ##
###########
add_library(${PROJECT_NAME} SHARED
  src/problem.cc
  src/composite.cc
  src/leaves.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIR})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)


#############
## Install ##
#############
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
#  DIRECTORY include/ifopt/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ifopt
#  FILES_MATCHING PATTERN "*.h"
#)


#############
## Testing ##
#############
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building IFOPT test executable \"testifopt\"")
  add_executable(testifopt test/composite_test.cc
                           test/problem_test.cc)
    
  target_link_libraries(testifopt PRIVATE 
    ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()
