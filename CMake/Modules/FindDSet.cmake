# Try to find DSet (https://github.com/wjakob/dset)
#
# Once done, this will define
#
#  DSet_FOUND
#  DSet_ROOT
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
#  DSet_ROOT

  
find_path(DSet_ROOT NAMES dset.h
	HINTS
	ENV DSet_ROOT 
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DSet DEFAULT_MSG DSet_ROOT)