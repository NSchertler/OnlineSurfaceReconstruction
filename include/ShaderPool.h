/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include "GLShader.h"

namespace osr
{
	//Singleton. Holds all shaders during the runtime of the application.
	class ShaderPool
	{
	private:
		static ShaderPool* _instance;
		ShaderPool();

	public:
		static ShaderPool* Instance();
		void CompileAll();

		gui::GLShader ObjectShader;
		gui::GLShader MeshColorsTriShader;
		gui::GLShader MeshColorsQuadShader;
		gui::GLShader TessellatedEdgesShader;
		gui::GLShader NormalShader;
		gui::GLShader OrientationFieldShader;
		gui::GLShader PositionFieldShader;
		gui::GLShader AdjacencyShader;
		gui::GLShader SphereShader;
		gui::GLShader PhantomShader;
	};
}