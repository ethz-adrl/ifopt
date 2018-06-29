###########
## Build ##
###########
add_library(${LIB_SNOPT} SHARED
  src/snopt.cc
  src/snopt_adapter.cc
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
    ifopt_core
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
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_snopt.so
install(TARGETS ${LIB_SNOPT} EXPORT ${LIB_SNOPT}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
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


#############
## Testing ##
#############
add_executable(${LIB_SNOPT}-example test/ex_test_snopt.cc)
target_link_libraries(${LIB_SNOPT}-example 
  PRIVATE 
    ${LIB_SNOPT}
)
add_test(${LIB_SNOPT}-example ${LIB_SNOPT}-example)