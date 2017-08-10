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

#include <nsessentials/gui/GLShader.h>

namespace osr
{
	namespace gui
	{
		//Singleton. Holds all shaders during the runtime of the application.
		class ShaderPool
		{
		private:
			static ShaderPool* _instance;
			ShaderPool();

			bool hasMeshColorSupport;

		public:
			static ShaderPool* Instance();
			void CompileAll();

			nse::gui::GLShader ObjectShader;
			nse::gui::GLShader MeshColorsTriShader;
			nse::gui::GLShader MeshColorsQuadShader;
			nse::gui::GLShader FlatMeshShader;
			nse::gui::GLShader TessellatedEdgesShader;
			nse::gui::GLShader NormalShader;
			nse::gui::GLShader OrientationFieldShader;
			nse::gui::GLShader PositionFieldShader;
			nse::gui::GLShader AdjacencyShader;
			nse::gui::GLShader SphereShader;
			nse::gui::GLShader PhantomShader;

			// Returns if the current OpenGL context supports Shader Storage Buffer Objects and Tessellation Shaders.
			// This function will return correct values only after CompileAll() has been called.
			bool HasMeshColorSupport() const;
		};
	}
}