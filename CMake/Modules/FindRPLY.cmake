# Try to find RPLY 
#
# Once done, this will define
#
#  RPLY_FOUND
#  RPLY_ROOT
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
#  RPLY_ROOT

  
find_path(RPLY_ROOT NAMES rply.h
	HINTS
	ENV RPLY_ROOT 
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(RPLY DEFAULT_MSG RPLY_ROOT)