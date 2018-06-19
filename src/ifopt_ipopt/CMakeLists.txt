cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_ipopt)

###########
## Build ##
###########
# The IPOPT solver interface
add_library(${PROJECT_NAME} SHARED src/ipopt_adapter.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${IPOPT_INCLUDE_DIRS} # need both these
  #${EIGEN3_INCLUDE_DIR} # need both these
  #$<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC  ifopt_core
  PRIVATE ${IPOPT_LIB}
)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_ipopt.so
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Copy header files to usr/local/include/ifopt/*
install(
  DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)


#############
## Testing ##
#############
add_executable(testipopt test/ex_test_ipopt.cc)
target_link_libraries(testipopt PRIVATE ${PROJECT_NAME})
