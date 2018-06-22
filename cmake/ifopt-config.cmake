# ifopt-config.cmake
# ---------
#
# Locates the optimization interface ifopt.
# Copyright (c) 2018, Alexander W. Winkler. All rights reserved.
#
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following IMPORTED targets:
#   ifopt::ifopt_core  - formulate a solver-independent optimization problem
#   ifopt::ifopt_ipopt - interface to NLP solver IPOPT
#   ifopt::ifopt_snopt - interface to NLP solver SNOPT 
#
#
# Example usage
# ^^^^^^^^^^^^^
#
#   find_package(ifopt)
#   add_executable(foo my_problem_solved_with_IPOPT.cc)
#   target_link_libraries(foo PUBLIC ifopt::ifopt_ipopt)
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project. 
# These can be neccessary when building with catkin and not modern cmake
#   ifopt_FOUND - TRUE if this project is found
#   ifopt_INCLUDE_DIRS - path to public include (.h) files
#   ifopt_ifopt_LIBRARIES - path to each of the libraries
#
#
# Additional information
# ^^^^^^^^^^^^^^^^
#
#   - Modern cmake: https://youtu.be/bsXLMQ6WgIk?t=2357
#   - How to auto-generated find scripts: 
#       https://cmake.org/cmake/help/v3.0/manual/cmake-packages.7.html
#
#=============================================================================


# these are autogenerate by cmake
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_core-targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_ipopt-targets.cmake" OPTIONAL)
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_snopt-targets.cmake" OPTIONAL)


# these are variables helpful to use when building with catkin
set(ifopt_FOUND TRUE)
 
get_target_property(ifopt_INCLUDE_DIRS ifopt::ifopt_core INTERFACE_INCLUDE_DIRECTORIES)

get_property(ifopt_LIBRARIES TARGET ifopt::ifopt_core PROPERTY LOCATION)
if(TARGET ifopt::ifopt_ipopt)
  get_property(path TARGET ifopt::ifopt_ipopt PROPERTY LOCATION)
  set(ifopt_LIBRARIES "${ifopt_LIBRARIES};${path}")
endif()
if(TARGET ifopt::ifopt_snopt)
  get_property(path TARGET ifopt::ifopt_snopt PROPERTY LOCATION)
  set(ifopt_LIBRARIES "${ifopt_LIBRARIES};${path}")
endif()

