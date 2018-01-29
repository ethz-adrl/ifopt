cmake_minimum_required(VERSION 2.8.3)
project(ifopt_core)

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
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark header files for installation
install(
  DIRECTORY include/ifopt/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
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
endif()

