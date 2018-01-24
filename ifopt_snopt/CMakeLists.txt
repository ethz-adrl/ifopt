cmake_minimum_required(VERSION 2.8.3)
project(ifopt_snopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS ifopt)

# adapt this to point to your installed Snopt folder
set(SNOPT_DIR "/home/winklera/3rd_party_software/snopt_lib")
# True if version SNOPT 7.6 or greater (interface changed -.-) 
set(SNOPT76 FALSE)

message(STATUS "SNOPT_DIR set to " ${SNOPT_DIR})
if(${SNOPT76})
  message(STATUS "SNOPT76 version detected")
  target_compile_definitions(snopt_test PRIVATE SNOPT76_ON=1)
endif()

set(SNOPT_INCLUDE_DIRS ${SNOPT_DIR}/include)
find_library(SNOPT_LIB1 snopt7_cpp ${SNOPT_DIR}/lib)
find_library(SNOPT_LIB2 snopt7 ${SNOPT_DIR}/lib)


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

if(${SNOPT76})
 add_library(${PROJECT_NAME} src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} src/snopt_adapter.cc)
endif()

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SNOPT_LIB1}
  ${SNOPT_LIB2}
)


#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_snopt.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
  )
endif()
