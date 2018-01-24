cmake_minimum_required(VERSION 2.8.3)
project(ifopt_ipopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS ifopt)

# adapt this to point to your installed IPOPT folder
set(IPOPT_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.8")
message(STATUS "IPOPT_DIR set to " ${IPOPT_DIR})
set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/build/include/coin)
set(IPOPT_LIBRARIES    ${IPOPT_DIR}/build/lib/libipopt.so)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS   include ${IPOPT_INCLUDE_DIRS}
  LIBRARIES      ${PROJECT_NAME}
  CATKIN_DEPENDS ifopt
)


###########
## Build ##
###########
include_directories(
  include
  ${IPOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# The IPOPT solver interface
add_library(${PROJECT_NAME} src/ipopt_adapter.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES}
)


#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_ipopt.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
  )
endif()