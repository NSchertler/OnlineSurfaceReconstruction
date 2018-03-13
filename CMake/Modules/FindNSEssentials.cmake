# Try to find NSEssentials
#
# Once done, this will define
#
#  NSEssentials_FOUND
#  NSEssentials_ROOT
#
# This module reads hints about search locations from 
# the following enviroment variables:
#
#  NSE_ROOT

  
find_path(NSE_ROOT NAMES include/nsessentials/data/FileHelper.h
	HINTS
	ENV NSE_ROOT 
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NSEssentials DEFAULT_MSG NSE_ROOT)