cmake_minimum_required (VERSION 3.1)
project(OnlineSurfaceReconstruction)

FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

FUNCTION(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
ENDFUNCTION()

option(OSR_USE_DAVIDVIVE "[Under Construction] Use the DAVID/Vive scan loader")
option(OSR_ACCURATE_TIMING "Display all timings in ms")
option(OSR_DEBUG_VISUALIZATION "Generate debug visualizations")
option(OSR_REDUCED_TIMINGS "Measure time only for high-priority nse::util::TimedBlocks")
option(OSR_GEOMETRIC_LAPLACIAN "Use the more time-consuming but higher-quality geometric formulation of the Laplacian for displacement map calculation")
option(OSR_USE_MEMORY_MAPPED_FILE "Use the boost memory-mapped file for XYZ input. This requires the boost iostreams library.")

set(NANOGUI_USE_GLAD ON)

set(ADDITIONAL_SRC)
set(LIBS)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Enable folders for projects in Visual Studio
if (CMAKE_GENERATOR MATCHES "Visual Studio")
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

# Sanitize build environment for static build with C++11
if (MSVC)
  add_definitions (/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions (/D "__TBB_NO_IMPLICIT_LINKAGE")

  add_definitions (-DNOMINMAX )

  # Parallel build on MSVC (all targets)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

  if (NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")

    # Disable Eigen vectorization for Windows 32 bit builds (issues with unaligned access segfaults)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DEIGEN_DONT_ALIGN")
  endif()

  # Static build
  set(CompilerFlags
        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO
        CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO)
  foreach(CompilerFlag ${CompilerFlags})
    string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
  endforeach()
elseif(APPLE)
  # Try to auto-detect a suitable SDK
  execute_process(COMMAND bash -c "xcodebuild -version -sdk | grep MacOSX | grep Path | head -n 1 | cut -f 2 -d ' '" OUTPUT_VARIABLE CMAKE_OSX_SYSROOT)
  string(REGEX REPLACE "(\r?\n)+$" "" CMAKE_OSX_SYSROOT "${CMAKE_OSX_SYSROOT}")
  string(REGEX REPLACE "^.*X([0-9.]*).sdk$" "\\1" CMAKE_OSX_DEPLOYMENT_TARGET "${CMAKE_OSX_SYSROOT}")
endif()

set (CMAKE_CXX_STANDARD 14)

# Build NanoGUI
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_SHARED  OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui ext_build/nanogui)
set_property(TARGET nanogui glfw glfw_objects PROPERTY FOLDER "dependencies")
include_directories(ext/nanogui/include
					${NANOGUI_EXTRA_INCS})
set(LIBS ${LIBS} nanogui ${NANOGUI_EXTRA_LIBS})
add_definitions(${NANOGUI_EXTRA_DEFS})
SET(NSE_WITH_NANOGUI ON CACHE BOOL " " FORCE)
SET(NSE_WITH_EIGEN ON CACHE BOOL " " FORCE)

# Build TBB
set(TBB_BUILD_STATIC ON CACHE BOOL " " FORCE)
set(TBB_BUILD_SHARED OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS OFF CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb ext_build/tbb)
set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")
include_directories(ext/tbb/include)
set(LIBS ${LIBS} tbb_static)
SET(NSE_WITH_TBB ON CACHE BOOL " " FORCE)

# Build NSEssentials
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nsessentials ext_build/nsessentials)
set_property(TARGET nsessentials PROPERTY FOLDER "dependencies")
include_directories(ext/nsessentials/include)
set(LIBS ${LIBS} nsessentials)
add_definitions(${NSE_EXTRA_DEFS})

set(BOOST_COMPONENTS)
if(OSR_USE_MEMORY_MAPPED_FILE)
	set(BOOST_COMPONENTS ${BOOST_COMPONENTS} iostreams)
	add_definitions(/DOSR_USE_MEMORY_MAPPED_FILE)
endif()

# Boost required for any and cstdint on MSVC < 2010
find_package(Boost COMPONENTS ${BOOST_COMPONENTS} REQUIRED)
if(MSVC AND (MSVC_VERSION LESS 1600))
	add_definitions(-DBOOST_STDINT)
endif(MSVC AND (MSVC_VERSION LESS 1600))
include_directories(${Boost_INCLUDE_DIRS})
if(NOT "${BOOST_COMPONENTS}" STREQUAL "")
	message("We have some boost components")
	link_directories(${Boost_LIBRARY_DIRS})
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  add_definitions(-DOPENMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#Do not use Eigen parallelization as we will parallelize manually
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")

if(OSR_ACCURATE_TIMING)
	add_definitions (/DACCURATE_TIMING)
endif(OSR_ACCURATE_TIMING)

if(OSR_DEBUG_VISUALIZATION)
	add_definitions(/DDEBUG_VISUALIZATION)
endif(OSR_DEBUG_VISUALIZATION)

if(OSR_REDUCED_TIMINGS)
	add_definitions(/DREDUCED_TIMINGS)
endif(OSR_REDUCED_TIMINGS)

if(OSR_GEOMETRIC_LAPLACIAN)
	add_definitions (/DGEOMETRIC_LAPLACIAN)
endif(OSR_GEOMETRIC_LAPLACIAN)

set(GLSL_FILES 	mesh.vert
				mesh_flat.vert
				blinnphong.frag
				flat_shading.geom
				void.vert
				mesh_colors_tri.tcs mesh_colors_tri.tes
				mesh_colors_quad.tcs mesh_colors_quad.tes
				edges.vert edges.tcs edges.tes
				orientation_field.frag orientation_field.geom orientation_field.vert
				normal.frag normal.geom normal.vert
				position_field.frag position_field.vert
				lines.vert lines.frag
				sphere.vert sphere.geom sphere.frag
				phantom.vert phantom.frag)

PREPEND(GLSL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/glsl/" ${GLSL_FILES})

source_group(glsl FILES ${GLSL_FILES})

JOIN("${GLSL_FILES}" "," glsl_string)
set(bin2c_cmdline
    -DOUTPUT_C=glsl.cpp
    -DOUTPUT_H=glsl.h
    "-DINPUT_FILES=${glsl_string}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/resources/bin2c.cmake")

add_custom_command(
  OUTPUT glsl.cpp glsl.h
  COMMAND ${CMAKE_COMMAND} ARGS ${bin2c_cmdline}
  DEPENDS ${GLSL_FILES}
  COMMENT "Running bin2c"
  PRE_BUILD VERBATIM)

SET(THIRD_PARTY 	ext/rply/rply.c
					include/3rd/ICP.h)
source_group(third_party FILES ${THIRD_PARTY})

SET(GUI_FILES	src/osr/gui/Viewer.cpp include/osr/gui/Viewer.h				
				include/osr/gui/GUIObject.h								
				src/osr/gui/HierarchyRenderer.cpp include/osr/gui/HierarchyRenderer.h
				src/osr/gui/ExtractedMeshGL.cpp include/osr/gui/ExtractedMeshGL.h
				src/osr/gui/ShaderPool.cpp include/osr/gui/ShaderPool.h)
source_group(gui FILES ${GUI_FILES})

SET(GUI_TOOL_FILES
				include/osr/gui/tools/Tool.h
				src/osr/gui/tools/FillHoleTool.cpp include/osr/gui/tools/FillHoleTool.h
				src/osr/gui/tools/SmoothTool.cpp include/osr/gui/tools/SmoothTool.h
				src/osr/gui/tools/RemoveTool.cpp include/osr/gui/tools/RemoveTool.h
				src/osr/gui/tools/ManualCoarseRegistrationTool.cpp include/osr/gui/tools/ManualCoarseRegistrationTool.h
				src/osr/gui/tools/Selection.cpp include/osr/gui/tools/Selection.h)
			
source_group(gui\\tools FILES ${GUI_TOOL_FILES})
			
SET(GUI_LOADERS_FILES
				include/osr/gui/loaders/ScanLoader.h
				src/osr/gui/loaders/FileScanLoader.cpp include/osr/gui/loaders/FileScanLoader.h
				src/osr/gui/loaders/ProceduralScanLoader.cpp include/osr/gui/loaders/ProceduralScanLoader.h)

if(OSR_USE_DAVIDVIVE)
	set(GUI_LOADERS_FILES ${GUI_LOADERS_FILES} 
		src/osr/gui/loaders/DavidViveScanLoader.cpp include/osr/gui/loaders/DavidViveScanLoader.h 
		src/osr/gui/loaders/DavidTurntableConnection.cpp include/osr/gui/loaders/DavidTurntableConnection.h)

	add_definitions (/DUSE_DAVIDVIVE)
	find_package(OpenVR REQUIRED)
	include_directories(${OPENVR_INCLUDE_DIRS})
	set(LIBS ${LIBS} ${OPENVR_LIB})

	set(DAVIDVIVE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/DavidVive/TakeDavidScan.au3" "${CMAKE_CURRENT_SOURCE_DIR}/DavidVive/viveController.ply")

	JOIN("${DAVIDVIVE_FILES}" "," davidvive_string)
	set(bin2c_cmdline
		-DOUTPUT_C=davidviveres.cpp
		-DOUTPUT_H=davidviveres.h
		"-DINPUT_FILES=${davidvive_string}"
		-P "${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/resources/bin2c.cmake")

	add_custom_command(
	  OUTPUT davidviveres.cpp davidviveres.h
	  COMMAND ${CMAKE_COMMAND} ARGS ${bin2c_cmdline}
	  DEPENDS ${DAVIDVIVE_FILES}
	  COMMENT "Running bin2c"
	  PRE_BUILD VERBATIM)

	set(ADDITIONAL_SRC ${ADDITIONAL_SRC} davidviveres.cpp)
endif(OSR_USE_DAVIDVIVE)

source_group(gui\\loaders FILES ${GUI_LOADERS_FILES})				

include_directories(
  include  
  # Resource file
  ${CMAKE_CURRENT_BINARY_DIR}
  # RPLY mesh I/O library
  ext/rply
  # Disjoint set data structure
  ext/dset
  # nanoflann
  ext/nanoflann
)

add_executable(OnlineSurfaceReconstruction MACOSX_BUNDLE
	glsl.cpp
	src/main.cpp
	include/osr/Attributes.h
	include/osr/AttributeConsistency.h
	src/osr/common.cpp include/osr/common.h
	src/osr/Data.cpp include/osr/Data.h
	include/osr/MeshSettings.h
	src/osr/field.cpp include/osr/field.h
	include/osr/Indexing.h
	include/osr/HierarchyOptions.h
	include/osr/HierarchyDecl.h
	include/osr/HierarchyDef.h
	include/osr/Hierarchy.h
	include/osr/PreparedVertexSet.h
	src/osr/filehelper.cpp include/osr/filehelper.h
	src/osr/HierarchyOptimizationHelper.cpp include/osr/HierarchyOptimizationHelper.h	
	#src/HierarchyUnstructured.cpp include/HierarchyUnstructured.h
	#src/HierarchyRegularSubdiv.cpp include/HierarchyRegularSubdiv.h
	#src/HierarchyOctree.cpp include/HierarchyOctree.h
	src/osr/HierarchyMortonMultiPass.cpp include/osr/HierarchyMortonMultiPass.h
	include/osr/HierarchyCapabilities.h
	include/osr/INeighborQueryable.h
	src/osr/meshio.cpp include/osr/meshio.h
	src/osr/Optimizer.cpp include/osr/Optimizer.h
	src/osr/Scan.cpp include/osr/Scan.h
	src/osr/ExtractedMesh.cpp include/osr/ExtractedMesh.h
	src/osr/MeshVisitor.cpp include/osr/MeshVisitor.h
	include/osr/ExtractionUtils.h
	src/osr/ExtractionAttributeMapping.cpp include/osr/ExtractionAttributeMapping.h		
	src/osr/BatchSession.cpp include/osr/BatchSession.h
	src/osr/Colors.cpp include/osr/Colors.h
	${GUI_FILES}
	${GUI_TOOL_FILES}
	${GUI_LOADERS_FILES}
	${ADDITIONAL_SRC}
	${GLSL_FILES}
	${THIRD_PARTY})

target_link_libraries(OnlineSurfaceReconstruction ${LIBS})