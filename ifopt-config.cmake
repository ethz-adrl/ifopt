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
#   ifopt::ifopt_core   - formulate a solver-independent optimization problem
#   (ifopt::ifopt_ipopt - interface to NLP solver IPOPT)
#   (ifopt::ifopt_snopt - interface to NLP solver SNOPT)
#
#
# Example usage
# ^^^^^^^^^^^^^
#
#   find_package(ifopt REQUIRED)
#   add_executable(foo my_problem_solved_with_IPOPT.cc)
#   target_link_libraries(foo PUBLIC ifopt::ifopt_ipopt)
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project. 
# These can be neccessary when building with catkin and not modern cmake
#   ifopt_INCLUDE_DIRS    - path to public include (.h) files
#   ifopt_LIBRARIES       - path to all libraries
#   ifopt_LIB_CORE        - path to ifopt_core library
#   (ifopt_LIB_IPOPT      - path to ifopt_ipopt library)
#   (ifopt_LIB_SNOPT      - path to ifopt_snopt library)
#
#=============================================================================

include(CMakeFindDependencyMacro)
find_dependency(Eigen3)

# these are autogenerate by cmake
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_core-targets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_ipopt-targets.cmake" OPTIONAL)
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_snopt-targets.cmake" OPTIONAL)

get_target_property(ifopt_INCLUDE_DIRS ifopt::ifopt_core INTERFACE_INCLUDE_DIRECTORIES)

# ifopt_core
get_property(ifopt_LIB_CORE TARGET ifopt::ifopt_core PROPERTY LOCATION)
list(APPEND ifopt_LIBRARIES ${ifopt_LIB_CORE})

# ifopt_ipopt
if(TARGET ifopt::ifopt_ipopt)
  get_property(ifopt_LIB_IPOPT TARGET ifopt::ifopt_ipopt PROPERTY LOCATION)
  list(APPEND ifopt_LIBRARIES ${ifopt_LIB_IPOPT})
endif()

# ifopt_snopt
if(TARGET ifopt::ifopt_snopt)
  get_property(ifopt_LIB_SNOPT TARGET ifopt::ifopt_snopt PROPERTY LOCATION)
  list(APPEND ifopt_LIBRARIES ${ifopt_LIB_SNOPT})
endif()

