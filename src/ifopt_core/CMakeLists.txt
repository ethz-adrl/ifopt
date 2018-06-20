cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_core)

###########
## Build ##
###########
add_library(${PROJECT_NAME} 
  SHARED
    src/problem.cc
    src/composite.cc
    src/leaves.cc
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)
# better specify features you need, not c++ version
target_compile_features(${PROJECT_NAME} 
  PUBLIC 
    cxx_range_for 
    cxx_auto_type
)


#############
## Install ##
#############
# store in export-set for find-script generation
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# Generate the find scripts
install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)


#############
## Testing ##
#############
find_package(GTest  QUIET)
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building IFOPT test executable \"testifopt\"")
  add_executable(testifopt 
    test/composite_test.cc
    test/problem_test.cc)
    
  target_link_libraries(testifopt 
    PRIVATE 
      ${PROJECT_NAME} 
      GTest::GTest GTest::Main)
endif()
