# Findsnopt.cmake
# ---------
#
# Basic helper script to locate the header files and libraries of SNOPT.
# Web: http://ampl.com/products/solvers/solvers-we-sell/snopt/
# Copyright (c) 2018, Alexander W. Winkler. All rights reserved.
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#   snopt_INCLUDE_DIRS - path to public SNOPT include (.h) files
#   snopt_LIBRARIES    - path to all snopt libraries
#   snopt_v76          - true if snopt version >= v7.6
#
#=============================================================================


# adapt this to point to your installed SNOPT folder
set(snopt_DIR "/home/winklera/3rd_party_software/snopt_lib")
set(snopt_v76 FALSE) # set true if snopt version >= v7.6

if(IS_DIRECTORY ${snopt_DIR}/include)
  message(STATUS "SNOPT found at:  \"" ${snopt_DIR} "\" ")
  set(LIB_SNOPT ifopt_snopt)
  set(snopt_INCLUDE_DIRS "${snopt_DIR}/include")
  set(snopt_LIBRARIES    "${snopt_DIR}/lib/libsnopt7_cpp.so;"
                         "${snopt_DIR}/lib/libsnopt7.so")
else()
  message(WARNING "SNOPT directory \"" ${snopt_DIR} "\" NOT found "
                  "-> Not compiling ifopt_snopt.\n" 
                  "Specify path to your installed SNOPT installation here.")
endif()
