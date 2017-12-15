# Solve example problem with IPOPT if installed
if(IPOPT_FOUND)
  add_executable(ex_ipopt ex_test_ipopt.cc)
  target_link_libraries(ex_ipopt
                        ${PROJECT_NAME}
                        ${IPOPT_ADAPTER_LIB}
                        ${IPOPT_LIBRARIES})
  add_test(ex_ipopt ex_ipopt)
endif()



# Solve example problem with SNOPT if installed
if(SNOPT_FOUND)
  add_executable(ex_snopt ex_test_snopt.cc)
  target_link_libraries(ex_snopt
                        ${PROJECT_NAME}
                        ${SNOPT_ADAPTER_LIB}
                        ${SNOPT_LIBRARIES})
  add_test(ex_snopt ex_snopt)
endif()



# Run some solver independent unit tests
find_package(GTest)
if(GTEST_FOUND)
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