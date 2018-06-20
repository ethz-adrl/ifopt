project(ifopt_ipopt)

###########
## Build ##
###########
# The IPOPT solver interface
add_library(${PROJECT_NAME} 
  SHARED 
    src/ipopt_adapter.cc
)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${IPOPT_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC 
    ifopt_core
    ${IPOPT_LIB}
)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_ipopt.so
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# install a *.cmake file in destination
install(EXPORT ${PROJECT_NAME}-targets
  FILE ${PROJECT_NAME}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)


#############
## Testing ##
#############
add_executable(testipopt test/ex_test_ipopt.cc)
target_link_libraries(testipopt 
  PRIVATE 
    ${PROJECT_NAME}
)
add_test(testipopt testipopt)
