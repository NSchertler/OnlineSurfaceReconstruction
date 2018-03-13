# Try to find the CMake port of TBB (https://github.com/wjakob/tbb)
#
# Once done, this will define
#
#  TBB_FOUND
#  TBB_ROOT
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
#  TBB_ROOT

  
find_path(TBB_ROOT NAMES include/tbb/aggregator.h
	HINTS
	ENV TBB_ROOT 
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TBB DEFAULT_MSG TBB_ROOT)