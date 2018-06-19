cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_snopt)

###########
## Build ##
###########
if(${SNOPT76})
 add_library(${PROJECT_NAME} SHARED src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} SHARED src/snopt_adapter.cc)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC 
  include
  ${SNOPT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ifopt_core
  ${SNOPT_LIB1}
  ${SNOPT_LIB2}
)


#############
## Install ##
#############
 # Copy library files to usr/local/lib/libifopt_snopt.so
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
add_executable(testsnopt test/ex_test_snopt.cc)
target_link_libraries(testsnopt PRIVATE ${PROJECT_NAME})