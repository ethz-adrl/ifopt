find_package(Eigen3 REQUIRED)


###########
## Build ##
###########
add_library(${LIB_CORE}
    src/problem.cc
    src/composite.cc
    src/leaves.cc
)
target_include_directories(${LIB_CORE} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${LIB_CORE} 
  PUBLIC
    Eigen3::Eigen
)
# better specify features you need, not c++ version
target_compile_features(${LIB_CORE} 
  PUBLIC 
    cxx_range_for 
    cxx_auto_type
)

#############
## Install ##
#############
# store in export-set for find-script generation
install(
  TARGETS ${LIB_CORE}
  EXPORT  ${LIB_CORE}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(
  DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
install(
  FILES test/ifopt/test_vars_constr_cost.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# Generate the find scripts
install(
  EXPORT ${LIB_CORE}-targets
  FILE ${LIB_CORE}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)


#############
## Testing ##
#############
find_package(GTest QUIET)
if (TARGET GTest::GTest) # only build when modern target exists
  add_executable(${LIB_CORE}-test 
    test/composite_test.cc
    test/problem_test.cc
  )  
  target_link_libraries(${LIB_CORE}-test  
    PRIVATE 
      ${LIB_CORE} 
      GTest::GTest GTest::Main
  )    
  add_test(${LIB_CORE}-test  ${LIB_CORE}-test)
  
  install(
    TARGETS ${LIB_CORE}-test
    RUNTIME DESTINATION ${IFOPT_INSTALL_BINDIR}
  )
endif()
