cmake_minimum_required(VERSION 2.8.3)
project(ifopt)
option(CATKIN_BUILD "CATKIN_BUILD" ON)
MESSAGE(STATUS "CATKIN_BUILD is " ${CATKIN_BUILD})

if(${CATKIN_BUILD})
    find_package(catkin REQUIRED)
    catkin_metapackage()
else()
    # do native cmake build
    # add gtest and gmock support
    # - see http://www.kaizou.org/2014/11/gtest-cmake/
    # ------------- BEGIN FROM BLOG ------------------
    # We need thread support
    find_package(Threads REQUIRED)
    
    # Enable ExternalProject CMake module
    include(ExternalProject)
    
    # Download and install GoogleTest
    ExternalProject_Add(
        gtest
        URL https://github.com/google/googletest/archive/master.zip
        PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
        # Disable install step
        INSTALL_COMMAND ""
    )
                        
    # Get GTest source and binary directories from CMake project
    ExternalProject_Get_Property(gtest source_dir binary_dir)
                        
    # Create a libgtest target to be used as a dependency by test programs
    add_library(libgtest IMPORTED STATIC GLOBAL)
    add_dependencies(libgtest gtest)
                        
    # Set libgtest properties
    set_target_properties(libgtest PROPERTIES
        "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
        "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
    )
                                
    # Create a libgmock target to be used as a dependency by test programs
    add_library(libgmock IMPORTED STATIC GLOBAL)
    add_dependencies(libgmock gtest)
                                
    # Set libgmock properties
    set_target_properties(libgmock PROPERTIES
        "IMPORTED_LOCATION" "${binary_dir}/googlemock/libgmock.a"
        "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
    )
                                        
    # I couldn't make it work with INTERFACE_INCLUDE_DIRECTORIES
    include_directories("${source_dir}/googletest/include"
        "${source_dir}/googlemock/include")
    # ------------- END FROM BLOG ------------------
    
    enable_testing()
    
    # add subdirectories
    add_subdirectory("../ifopt_core" "../../ifopt_core/build")
    add_subdirectory("../ifopt_ipopt" "../../ifopt_ipopt/build")
    add_subdirectory("../ifopt_snopt" "../../ifopt_snopt/build")
endif()
