# Run some solver independent unit tests
# the pure cmake way to do testing
find_package(GTest)

# the catkin way to do testing
if (CATKIN_ENABLE_TESTING)
  message(STATUS "building tests the catkin way")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  catkin_add_gtest(${PROJECT_NAME}_test
                   gtest_main.cc
                   composite_test.cc
                   problem_test.cc)
                   
  target_link_libraries(${PROJECT_NAME}_test
                        ${PROJECT_NAME})
                        
elseif(GTEST_FOUND)
  message(STATUS "building tests the pure cmake way")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  
  include_directories(${GTEST_INCLUDE_DIRS})
  
  add_executable(${PROJECT_NAME}_test
                 gtest_main.cc 
                 composite_test.cc
                 problem_test.cc)
  
  target_link_libraries(${PROJECT_NAME}_test
                        ${PROJECT_NAME}
                        ${GTEST_BOTH_LIBRARIES})
  
  add_test(gtest_all ${PROJECT_NAME}_test)
  
endif()                 


# Solve example problem with IPOPT if installed
if(IPOPT_FOUND)
  add_executable(ipopt_test ex_test_ipopt.cc)
  target_link_libraries(ipopt_test
                        ${PROJECT_NAME}
                        ${IPOPT_ADAPTER_LIB}
                        ${IPOPT_LIBRARIES})
  add_test(ipopt_test ipopt_test)
endif()



# Solve example problem with SNOPT if installed
if(SNOPT_FOUND)
  add_executable(snopt_test ex_test_snopt.cc)
  target_link_libraries(snopt_test
                        ${PROJECT_NAME}
                        ${SNOPT_ADAPTER_LIB}
                        ${SNOPT_LIBRARIES})
  add_test(snopt_test snopt_test)
endif()