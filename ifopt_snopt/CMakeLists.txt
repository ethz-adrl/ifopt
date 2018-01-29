cmake_minimum_required(VERSION 2.8.3)
project(ifopt_snopt)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(ifopt_core REQUIRED)


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
catkin_package(
  INCLUDE_DIRS include ${SNOPT_INCLUDE_DIRS} ${ifopt_core_INCLUDE_DIRS}
  LIBRARIES    ${PROJECT_NAME}
)


###########
## Build ##
###########
include_directories(
  include
  ${SNOPT_INCLUDE_DIRS}
  ${ifopt_core_INCLUDE_DIRS}
)

if(${SNOPT76})
 add_library(${PROJECT_NAME} src/snopt76_adapter.cc)
else()
 add_library(${PROJECT_NAME} src/snopt_adapter.cc)
endif()

target_link_libraries(${PROJECT_NAME}
  ${ifopt_core_LIBRARIES}
  ${SNOPT_LIB1}
  ${SNOPT_LIB2}
)


#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
catkin_add_gtest(${PROJECT_NAME}-test test/ex_test_snopt.cc)

if(${SNOPT76})
  target_compile_definitions(${PROJECT_NAME}-test PRIVATE SNOPT76_ON=1)
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    pthread
  )
endif()
endif()
