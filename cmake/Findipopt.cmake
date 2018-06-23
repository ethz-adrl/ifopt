# Findipopt.cmake
# ---------
#
# Basic helper script to locate the header files and libraries of IPOPT.
# Web: https://projects.coin-or.org/Ipopt
# Copyright (c) 2018, Alexander W. Winkler. All rights reserved.
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#   ipopt_INCLUDE_DIRS - path to public SNOPT include (.h) files
#   ipopt_LIBRARIES    - path to all snopt libraries
#
#=============================================================================

# adapt this to point to your installed IPOPT folder
set(ipopt_DIR "/home/winklera/3rd_party_software/Ipopt-3.12.8")

if(IS_DIRECTORY ${ipopt_DIR}/build/include/coin)
  message(STATUS "IPOPT found at:  \"" ${ipopt_DIR} "\" ")
  set(LIB_IPOPT ifopt_ipopt)
  set(ipopt_INCLUDE_DIRS "${ipopt_DIR}/build/include/coin")
  set(ipopt_LIBRARIES    "${ipopt_DIR}/build/lib/libipopt.so")
  
else()
  message(WARNING "IPOPT directory \"" ${ipopt_DIR} "\" NOT found " 
                  "-> Not compiling ifopt_ipopt. \n" 
                  "Specify path to your installed IPOPT installation here.")
endif()