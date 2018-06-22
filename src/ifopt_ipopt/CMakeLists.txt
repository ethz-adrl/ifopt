###########
## Build ##
###########
# The IPOPT solver interface
add_library(${LIB_IPOPT} 
  SHARED 
    src/ipopt.cc
    src/ipopt_adapter.cc
)
target_include_directories(${LIB_IPOPT} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${IPOPT_DIR}/build/include/coin
)
target_link_libraries(${LIB_IPOPT}
  PUBLIC 
    ifopt_core
  PRIVATE
    ${IPOPT_DIR}/build/lib/libipopt.so
)

#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_ipopt.so
install(TARGETS ${LIB_IPOPT} EXPORT ${LIB_IPOPT}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
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


#############
## Testing ##
#############
add_executable(testipopt test/ex_test_ipopt.cc)
target_link_libraries(testipopt 
  PRIVATE 
    ${LIB_IPOPT}
)
add_test(testipopt testipopt)
