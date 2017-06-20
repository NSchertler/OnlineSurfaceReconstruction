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

#include <glad/glad.h>

namespace osr {
	namespace gui
	{
		//Represents an OpenGL Vertex Array Object
		class GLVertexArray
		{
		public:
			GLVertexArray();
			~GLVertexArray();

			//Generates the VAO if it has not been generated yet.
			void generate();

			//Binds the VAO.
			void bind() const;

			//Binds a zero VAO.
			void unbind() const;

			//Returns if this VAO has been generated.
			bool valid() const;

		private:
			GLuint vaoId;
		};
	}
}