cmake_minimum_required (VERSION 2.9)
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

option(OSR_USE_DAVIDVIVE "Use the DAVID/Vive scan loader")
option(OSR_ACCURATE_TIMING "Display all timings in ms")
option(OSR_DEBUG_VISUALIZATION "Generate debug visualizations")
option(OSR_REDUCED_TIMINGS "Measure time only for high-priority TimedBlocks")
option(OSR_GEOMETRIC_LAPLACIAN "Use the more time-consuming but higher-quality geometric formulation of the Laplacian for displacement map calculation")

set(ADDITIONAL_SRC)
set(ADDITIONAL_INCLUDE)
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

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  # Enable C++11 mode on GCC / Clang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

# Build NanoGUI
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_SHARED  OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui ext_build/nanogui)
set_property(TARGET nanogui glfw glfw_objects PROPERTY FOLDER "dependencies")
set(ADDITIONAL_INCLUDE 	${ADDITIONAL_INCLUDE}
						${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/include
						${NANOGUI_EXTRA_INCS}  )
set(LIBS ${LIBS} nanogui ${NANOGUI_EXTRA_LIBS})

# Build TBB
set(TBB_BUILD_STATIC ON CACHE BOOL " " FORCE)
set(TBB_BUILD_SHARED OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS OFF CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb ext_build/tbb)
set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")
set(ADDITIONAL_INCLUDE 	${ADDITIONAL_INCLUDE}
						${CMAKE_CURRENT_SOURCE_DIR}/ext/tbb/include  )
set(LIBS ${LIBS} tbb_static)

# Boost required for any and cstdint on MSVC < 2010
find_package(Boost COMPONENTS filesystem REQUIRED)
if(MSVC AND (MSVC_VERSION LESS 1600))
	add_definitions(-DBOOST_STDINT)
endif(MSVC AND (MSVC_VERSION LESS 1600))
set(ADDITIONAL_INCLUDE 	${ADDITIONAL_INCLUDE}
						${Boost_INCLUDE_DIRS} )
link_directories(${Boost_LIBRARY_DIRS})

find_package(OpenMP)
if(OPENMP_FOUND)
  add_definitions(-DOPENMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if(OSR_USE_DAVIDVIVE)
	set(ADDITIONAL_SRC ${ADDITIONAL_SRC} src/DavidViveScanLoader.cpp include/DavidViveScanLoader.h src/DavidTurntableConnection.cpp include/DavidTurntableConnection.h)
	add_definitions (/DUSE_DAVIDVIVE)
	find_package(OpenVR REQUIRED)
	set(ADDITIONAL_INCLUDE ${ADDITIONAL_INCLUDE} ${OPENVR_INCLUDE_DIRS})
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

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ADDITIONAL_INCLUDE}
  # Resource file
  ${CMAKE_CURRENT_BINARY_DIR}
  # RPLY mesh I/O library
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/rply
  # Disjoint set data structure
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/dset
  # nanoflann
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanoflann
  # glad
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/nanogui/ext/glad/include
)

set(GLSL_FILES 	mesh.vert
				blinnphong.frag
				void.vert
				mesh_colors_tri.tcs mesh_colors_tri.geom mesh_colors_tri.tes
				mesh_colors_quad.tcs mesh_colors_quad.geom mesh_colors_quad.tes
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
					include/ICP.h)
source_group(third_party FILES ${THIRD_PARTY})

SET(GUI_FILES	src/AbstractViewer.cpp include/AbstractViewer.h
				src/Camera.cpp include/Camera.h
				src/Viewer.cpp include/Viewer.h
				include/ScanLoader.h
				src/FileScanLoader.cpp include/FileScanLoader.h
				src/ProceduralScanLoader.cpp include/ProceduralScanLoader.h
				include/GUIObject.h
				include/Tool.h
				src/FillHoleTool.cpp include/FillHoleTool.h
				src/SmoothTool.cpp include/SmoothTool.h
				src/RemoveTool.cpp include/RemoveTool.h
				src/ManualCoarseRegistrationTool.cpp include/ManualCoarseRegistrationTool.h
				src/Selection.cpp include/Selection.h
				src/GLBuffer.cpp include/GLBuffer.h
				src/GLShader.cpp include/GLShader.h
				src/GLVertexArray.cpp include/GLVertexArray.h)
source_group(gui FILES ${GUI_FILES})

add_executable(OnlineSurfaceReconstruction MACOSX_BUNDLE
	glsl.cpp
	src/main.cpp
	include/Attributes.h
	include/AttributeConsistency.h
	src/common.cpp include/common.h
	src/Timer.cpp include/Timer.h
	src/BoundingBox.cpp include/BoundingBox.h
	include/ParallelCG.h
	src/Parallelization.cpp include/Parallelization.h
	src/Data.cpp include/Data.h
	include/MeshSettings.h
	src/field.cpp include/field.h
	src/LeastSquaresSystem.cpp include/LeastSquaresSystem.h
	include/Indexing.h
	include/HierarchyOptions.h
	include/HierarchyDecl.h
	include/HierarchyDef.h
	include/Hierarchy.h
	include/PreparedVertexSet.h
	src/HierarchyOptimizationHelper.cpp include/HierarchyOptimizationHelper.h
	src/HierarchyRenderer.cpp include/HierarchyRenderer.h
	#src/HierarchyUnstructured.cpp include/HierarchyUnstructured.h
	#src/HierarchyRegularSubdiv.cpp include/HierarchyRegularSubdiv.h
	#src/HierarchyOctree.cpp include/HierarchyOctree.h
	src/HierarchyMortonMultiPass.cpp include/HierarchyMortonMultiPass.h
	include/HierarchyCapabilities.h
	src/IndentationLog.cpp include/IndentationLog.h
	include/PersistentIndexContainer.h
	src/Morton.cpp include/Morton.h
	include/INeighborQueryable.h
	src/meshio.cpp include/meshio.h
	src/Optimizer.cpp include/Optimizer.h
	src/Scan.cpp include/Scan.h
	src/ExtractedMesh.cpp include/ExtractedMesh.h
	include/ExtractionUtils.h
	src/ExtractionAttributeMapping.cpp include/ExtractionAttributeMapping.h
	src/ExtractedMeshGL.cpp include/ExtractedMeshGL.h
	src/ShaderPool.cpp include/ShaderPool.h
	src/BatchSession.cpp include/BatchSession.h
	src/Colors.cpp include/Colors.h
	${GUI_FILES}
	${ADDITIONAL_SRC}
	${GLSL_FILES}
	${THIRD_PARTY})

add_definitions(${NANOGUI_EXTRA_DEFS})
target_link_libraries(OnlineSurfaceReconstruction ${LIBS})

set_target_properties(OnlineSurfaceReconstruction PROPERTIES OUTPUT_NAME "Online Surface Reconstruction")
