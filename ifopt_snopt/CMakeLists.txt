cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(ifopt_snopt)

#find_package(catkin REQUIRED)
#find_package(ifopt_core REQUIRED)


################
## Find SNOPT ##
################
# adapt this to point to your installed IPOPT folder
set(SNOPT_DIR "/home/winklera/3rd_party_software/snopt_lib")
if(IS_DIRECTORY ${SNOPT_DIR}/include)
  set(SNOPT_INCLUDE_DIRS ${SNOPT_DIR}/include)
  find_library(SNOPT_LIB1 snopt7_cpp ${SNOPT_DIR}/lib)
  find_library(SNOPT_LIB2 snopt7 ${SNOPT_DIR}/lib)
else()
  message(WARNING "SNOPT directory \"" ${SNOPT_DIR} "\" NOT found "
                  "-> Not compiling ifopt_snopt.\n" 
                  "Specify path to your installed SNOPT installation here.")
  return()
endif()

# adapt depending on your SNOPT version
set(SNOPT76 FALSE)
if(${SNOPT76})
  message("SNOPT76 version >= SNOPT 7.6 detected")
endif()


#################################
## Generate cmake config files ##
#################################
# These can then be used to "find_package(ifopt_snopt)"
#catkin_package(
#  INCLUDE_DIRS include ${SNOPT_INCLUDE_DIRS} ${ifopt_core_INCLUDE_DIRS}
#  LIBRARIES    ${PROJECT_NAME}
#)


###########
## Build ##
###########
if(${SNOPT76})
 add_library(${PROJECT_NAME} STATIC src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} STATIC src/snopt_adapter.cc)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC 
  include
  ${SNOPT_INCLUDE_DIRS}
  ${ifopt_core_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ifopt_core
  ${SNOPT_LIB1}
  ${SNOPT_LIB2}
)


#############
## Install ##
#############
# For this to work, first make sure IPOPT headers and library are installed
# somewhere in the default search paths.
# Mark library for installation
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
## Mark header files for installation
#install(
#  DIRECTORY include/ifopt_snopt/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)


#############
## Testing ##
#############
find_package(GTest QUIET)
if (GTEST_FOUND)
  message(STATUS "GTest found -> Building SNOPT test executable \"testsnopt\"")
  add_executable(testsnopt test/ex_test_snopt.cc)
    
  target_link_libraries(testsnopt PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
endif()
