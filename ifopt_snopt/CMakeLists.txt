cmake_minimum_required(VERSION 2.8.3)
project(ifopt_snopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS ifopt)


## the environmental variable is set in the ~/.bashrc, e.g.
## export SNOPT_DIR=/home/path/to/snopt/snopt_lib
## for compatibility with SNOPT 7.6. please make sure to set the variable
## export SNOPT76 = true
##################################
set(SNOPT_DIR "/home/winklera/3rd_party_software/snopt_lib")
message(STATUS "SNOPT_DIR set to " $ENV{SNOPT_DIR})
if($ENV{SNOPT76})
  message(STATUS "SNOPT76 version detected")
endif()
set(SNOPT_INCLUDE_DIRS $ENV{SNOPT_DIR}/include)
find_library(SNOPT_LIB1 snopt7_cpp $ENV{SNOPT_DIR}/lib)
find_library(SNOPT_LIB2 snopt7 $ENV{SNOPT_DIR}/lib)
set(SNOPT_LIBRARIES ${SNOPT_LIB1} ${SNOPT_LIB2})


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS   include
                 ${SNOPT_INCLUDE_DIRS}
  LIBRARIES      ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt
)

###########
## Build ##
###########
include_directories(
  include
  ${SNOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if($ENV{SNOPT76})
 add_library(${PROJECT_NAME} src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} src/snopt_adapter.cc)
endif()

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SNOPT_LIBRARIES}
)


#############
## Testing ##
#############
if($ENV{SNOPT76})
  target_compile_definitions(snopt_test PRIVATE SNOPT76_ON=1)
endif()
  
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_snopt.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
  )
endif()
