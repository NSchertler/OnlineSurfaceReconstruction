# Try to find nanoflann (https://github.com/jlblancoc/nanoflann)
#
# Once done, this will define
#
#  NANOFLANN_FOUND
#  NANOFLANN_ROOT
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
#  NANOFLANN_ROOT

  
find_path(NANOFLANN_ROOT NAMES nanoflann.hpp
	HINTS
	ENV NANOFLANN_ROOT 
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Nanoflann DEFAULT_MSG NANOFLANN_ROOT)