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

target_include_directories(${PROJECT_NAME} PUBLIC include)
#target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)
# better specify features you need, not c++ version
target_compile_features(${PROJECT_NAME} 
  PUBLIC cxx_range_for cxx_auto_type)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC Eigen)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_core.so
#install(TARGETS ${PROJECT_NAME} EXPORT IfoptTargets
#  ARCHIVE DESTINATION  lib
#  LIBRARY DESTINATION  lib
#  RUNTIME DESTINATION  bin
#  INCLUDES DESTINATION include/ifopt
#)
#
## Copy header files to usr/local/include/ifopt/*
#install(EXPORT IfoptTargets
#  FILE IfoptTargets.cmake
#  NAMESPACE Ifopt::
#  DESTINATION lib/cmake/Ifopt
#)


#############
## Testing ##
#############
find_package(GTest  QUIET) # optional: enables/disables testing
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building IFOPT test executable \"testifopt\"")
  add_executable(testifopt test/composite_test.cc
                           test/problem_test.cc)
    
  target_link_libraries(testifopt PRIVATE 
    ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()
