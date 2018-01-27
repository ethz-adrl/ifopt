cmake_minimum_required(VERSION 2.8.3)
project(ifopt_ipopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(ifopt_core  REQUIRED)


################
## Find IPOPT ##
################
# adapt this to point to your installed IPOPT folder
set(IPOPT_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.8")
if(IS_DIRECTORY ${IPOPT_DIR}/build/include/coin)
  set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/build/include/coin)
  set(IPOPT_LIBRARIES    ${IPOPT_DIR}/build/lib/libipopt.so)
else()
  message(WARNING "IPOPT directory \"" ${IPOPT_DIR} "\" NOT found " 
                  "-> Not compiling ifopt_ipopt. \n" 
                  "Specify path to your installed IPOPT installation here.")
  return()
endif()



#################################
## Generate cmake config files ##
#################################
# These can then be used to "find_package(ifopt_ipopt)"
catkin_package(
  INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS} ${ifopt_core_INCLUDE_DIRS}
  LIBRARIES    ${PROJECT_NAME}
)


###########
## Build ##
###########
include_directories(
  include
  ${IPOPT_INCLUDE_DIRS}
  ${ifopt_core_INCLUDE_DIRS}
)

# The IPOPT solver interface
add_library(${PROJECT_NAME} src/ipopt_adapter.cc)
target_link_libraries(${PROJECT_NAME}
  ${ifopt_core_LIBRARIES}
  ${IPOPT_LIBRARIES}
)


#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_ipopt.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    pthread
  )
endif()