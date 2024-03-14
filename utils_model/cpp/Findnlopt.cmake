# Findnlopt.cmake
# Locate nlopt library and include directories

# Try to find nlopt library
# Once done, this will define:
#  NLOPT_FOUND - System has nlopt
#  NLOPT_INCLUDE_DIRS - The nlopt include directories
#  NLOPT_LIBRARIES - The nlopt libraries
#
# Based on FindPackageHandleStandardArgs module
# (https://cmake.org/cmake/help/latest/module/FindPackageHandleStandardArgs.html)

find_path(NLOPT_INCLUDE_DIRS NAMES nlopt.h PATH_SUFFIXES nlopt)
find_library(NLOPT_LIBRARIES NAMES nlopt)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(nlopt DEFAULT_MSG NLOPT_LIBRARIES NLOPT_INCLUDE_DIRS)

mark_as_advanced(NLOPT_LIBRARIES NLOPT_INCLUDE_DIRS)
