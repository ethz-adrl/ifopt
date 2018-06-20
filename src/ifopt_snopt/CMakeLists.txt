project(ifopt_snopt)

###########
## Build ##
###########
if(${SNOPT76})
 add_library(${PROJECT_NAME} SHARED src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} SHARED src/snopt_adapter.cc)
endif()

target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SNOPT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    ifopt_core
    ${SNOPT_LIB1}
    ${SNOPT_LIB2}
)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_snopt.so
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
add_executable(testsnopt test/ex_test_snopt.cc)
target_link_libraries(testsnopt 
  PRIVATE 
    ${PROJECT_NAME}
)
add_test(testsnopt testsnopt)