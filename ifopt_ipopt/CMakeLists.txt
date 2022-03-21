set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(IPOPT 3.11.9 REQUIRED)
# previous IPOPT versions have a bug which causes mumps segfault:
# https://bugs.launchpad.net/ubuntu/+source/coinor-ipopt/+bug/1167585


###########
## Build ##
###########
set(LIB_IPOPT ifopt_ipopt)
# The IPOPT solver interface
add_library(${LIB_IPOPT} 
    src/ipopt_adapter.cc
    src/ipopt_solver.cc
)
target_include_directories(${LIB_IPOPT} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${IPOPT_INCLUDE_DIRS}
)
target_link_libraries(${LIB_IPOPT}
  PUBLIC 
    ${LIB_CORE}
  PRIVATE
    ${IPOPT_LIBRARIES}
    ${IPOPT_LINK_FLAGS}
)
target_compile_definitions(${LIB_IPOPT}
  PRIVATE
    ${IPOPT_DEFINITIONS}
)


#############
## Testing ##
#############
add_executable(${LIB_IPOPT}-example test/ex_test_ipopt.cc)
target_link_libraries(${LIB_IPOPT}-example 
  PRIVATE 
    ${LIB_IPOPT}
)
add_test(${LIB_IPOPT}-example ${LIB_IPOPT}-example)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_ipopt.so
install(
  TARGETS ${LIB_IPOPT}
  EXPORT  ${LIB_IPOPT}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
# install example executable
install(
  TARGETS ${LIB_IPOPT}-example
  RUNTIME DESTINATION ${IFOPT_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# install a *.cmake file in destination
install(EXPORT ${LIB_IPOPT}-targets
  FILE ${LIB_IPOPT}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)



