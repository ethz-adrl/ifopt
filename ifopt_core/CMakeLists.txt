cmake_minimum_required(VERSION 2.8.3)
project(ifopt_core)
option(CATKIN_BUILD "CATKIN_BUILD" ON)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
if(${CATKIN_BUILD})
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
      LIBRARIES    ${PROJECT_NAME}
    )
endif()

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
if(${CATKIN_BUILD})
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
      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/ifopt
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
else()
    # cmake native build
    file(GLOB CORE_HFILES include/ifopt/*.h)
    install(FILES ${CORE_HFILES}  DESTINATION include)
    install(TARGETS ${PROJECT_NAME} DESTINATION lib)
    
    #############
    ## Testing ##
    #############
    file(GLOB CORE_TEST test/*.cc)
    add_executable(${PROJECT_NAME}-test ${CORE_TEST})
    target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME}
        libgtest
        libgmock
        )
    add_test(
        NAME ${PROJECT_NAME}-test
        COMMAND ${PROJECT_NAME}-test
        )
endif()
