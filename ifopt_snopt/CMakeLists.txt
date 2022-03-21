set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
find_package(SNOPT REQUIRED)

###########
## Build ##
###########
set(LIB_SNOPT ifopt_snopt)
add_library(${LIB_SNOPT}
  src/snopt_adapter.cc
  src/snopt_solver.cc
)
target_include_directories(${LIB_SNOPT} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${SNOPT_INCLUDE_DIRS}
)
target_link_libraries(${LIB_SNOPT} 
  PUBLIC
    ${LIB_CORE}
  PRIVATE  
    ${SNOPT_LIBRARIES}
)
if(${SNOPT_v76})
  message(STATUS "compiling for snopt version >= 7.6")
  target_compile_definitions(${LIB_SNOPT} 
    PRIVATE
      -DSNOPT76
  )
endif()


#############
## Testing ##
#############
add_executable(${LIB_SNOPT}-example test/ex_test_snopt.cc)
target_link_libraries(${LIB_SNOPT}-example 
  PRIVATE 
    ${LIB_SNOPT}
)
add_test(${LIB_SNOPT}-example ${LIB_SNOPT}-example)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_snopt.so
install(
  TARGETS ${LIB_SNOPT}
  EXPORT ${LIB_SNOPT}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# install sample executable
install(
  TARGETS ${LIB_SNOPT}-example
  RUNTIME DESTINATION ${IFOPT_INSTALL_BINDIR}
)
# Copy header files to usr/local/include/ifopt/*
install(DIRECTORY include/ifopt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ifopt
)
# Generate the find scripts
install(EXPORT ${LIB_SNOPT}-targets
  FILE ${LIB_SNOPT}-targets.cmake
  NAMESPACE ifopt::
  DESTINATION ${config_package_location}
)


