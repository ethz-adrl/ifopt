cmake_minimum_required(VERSION 2.8.3)
project(ifopt_ipopt)

add_compile_options(-std=c++11)

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

if(${CATKIN_BUILD})
    find_package(catkin REQUIRED)
    find_package(ifopt_core  REQUIRED)
    #################################
    ## Generate cmake config files ##
    #################################
    # These can then be used to "find_package(ifopt_ipopt)"
    catkin_package(
        INCLUDE_DIRS include ${IPOPT_INCLUDE_DIRS} ${ifopt_core_INCLUDE_DIRS}
        LIBRARIES    ${PROJECT_NAME}
    )
endif()

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
## Install ##
#############
# For this to work, first make sure IPOPT headers and library are installed
# somewhere in the default search paths.
# Mark library for installation
if(${CATKIN_BUILD})
    install(
      TARGETS ${PROJECT_NAME}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    # Mark header files for installation
    install(
      DIRECTORY include/ifopt_ipopt/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
    )

    #############
    ## Testing ##
    #############
    if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_ipopt.cc)
    if(TARGET ${PROJECT_NAME}-test)
      target_link_libraries(${PROJECT_NAME}-test 
        ${PROJECT_NAME}
        pthread
      )
    endif()
    endif()
else()
    # cmake native build
    file(GLOB IPOPT_HFILES include/ifopt_ipopt/*.h)
    install(FILES ${IPOPT_HFILES} DESTINATION include)
    install(TARGETS ${PROJECT_NAME} DESTINATION lib)

    #############
    ## Testing ##
    #############
    file(GLOB IPOPT_TEST test/*.cc)
    add_executable(${PROJECT_NAME}-test ${IPOPT_TEST})
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
