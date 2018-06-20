# https://youtu.be/bsXLMQ6WgIk?t=2357
# https://cmake.org/cmake/help/v3.0/manual/cmake-packages.7.html

include("${CMAKE_CURRENT_LIST_DIR}/ifopt_core-targets.cmake")

# these files only exist if build with ipopt or snopt. If not, the cmake call 
#   target_link_libraries(main ifopt::ipopt) 
# will fail.
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_ipopt-targets.cmake" OPTIONAL)
include("${CMAKE_CURRENT_LIST_DIR}/ifopt_snopt-targets.cmake" OPTIONAL)