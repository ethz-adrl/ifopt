cmake_minimum_required(VERSION 2.8.3)
project(ifopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES    ${PROJECT_NAME}
)


###########
## Build ##
###########
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/problem.cc
  src/composite.cc
  src/leaves.cc
)


#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}-test
                 test/gtest_main.cc
                 test/composite_test.cc
                 test/problem_test.cc)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    pthread
  )
endif()

