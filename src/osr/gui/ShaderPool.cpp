/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/ShaderPool.h"

#include "glsl.h"

using namespace osr;
using namespace osr::gui;

ShaderPool* ShaderPool::_instance(nullptr);

ShaderPool::ShaderPool()
{
}

ShaderPool* ShaderPool::Instance()
{
	if (_instance == nullptr)
		_instance = new ShaderPool();
	return _instance;
}

void ShaderPool::CompileAll()
{
	ObjectShader.init("MeshShader", std::string((const char*)mesh_vert, mesh_vert_size), std::string((const char*)blinnphong_frag, blinnphong_frag_size));
	MeshColorsTriShader.initWithTessellation("MeshColorsTriShader", std::string((const char*)void_vert, void_vert_size), std::string((const char*)mesh_colors_tri_tcs, mesh_colors_tri_tcs_size), std::string((const char*)mesh_colors_tri_tes, mesh_colors_tri_tes_size), std::string((const char*)blinnphong_frag, blinnphong_frag_size), std::string((const char*)mesh_colors_tri_geom, mesh_colors_tri_geom_size));
	MeshColorsQuadShader.initWithTessellation("MeshColorsQuadShader", std::string((const char*)void_vert, void_vert_size), std::string((const char*)mesh_colors_quad_tcs, mesh_colors_quad_tcs_size), std::string((const char*)mesh_colors_quad_tes, mesh_colors_quad_tes_size), std::string((const char*)blinnphong_frag, blinnphong_frag_size), std::string((const char*)mesh_colors_quad_geom, mesh_colors_quad_geom_size));
	TessellatedEdgesShader.initWithTessellation("EdgeShader", (const char*)edges_vert, std::string((const char*)edges_tcs, edges_tcs_size), std::string((const char*)edges_tes, edges_tes_size), std::string((const char*)lines_frag, lines_frag_size));
	NormalShader.init("normalShader", std::string((const char*)normal_vert, normal_vert_size), std::string((const char*)normal_frag, normal_frag_size), std::string((const char*)normal_geom, normal_geom_size));
	OrientationFieldShader.init("orientationFieldShader", std::string((const char*)orientation_field_vert, orientation_field_vert_size), std::string((const char*)orientation_field_frag, orientation_field_frag_size), std::string((const char*)orientation_field_geom, orientation_field_geom_size));	
	PositionFieldShader.init("positionFieldShader", std::string((const char*)position_field_vert, position_field_vert_size), std::string((const char*)position_field_frag, position_field_frag_size));
	AdjacencyShader.init("adjacency shader", std::string((const char*)lines_vert, lines_vert_size), std::string((const char*)lines_frag, lines_frag_size));
	SphereShader.init("sphere shader", std::string((const char*)sphere_vert, sphere_vert_size), std::string((const char*)sphere_frag, sphere_frag_size), std::string((const char*)sphere_geom, sphere_geom_size));
	PhantomShader.init("phantom shader", std::string((const char*)phantom_vert, phantom_vert_size), std::string((const char*)phantom_frag, phantom_frag_size));
}
