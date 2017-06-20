/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#pragma once

#include <nanogui/glutil.h>

namespace osr {
	namespace gui
	{

		enum GLBufferType
		{
			VertexBuffer,
			IndexBuffer,
			ShaderStorageBuffer
		};

		//Represents a generic OpenGL buffer
		class GLBuffer
		{
		public:
			GLBuffer(GLBufferType type);
			~GLBuffer();

			//Binds the buffer to the correct target slot.
			//If the buffer is not generated yet, a new buffer is generated.
			void bind();
			//Calls the according OpenGL function; mostly used for SSBOs
			void bindBufferBase(unsigned int base) const;

			/// Upload data from an Eigen matrix into the buffer
			template <typename Matrix>
			GLBuffer& uploadData(const Matrix &M)
			{
				uint32_t compSize = sizeof(typename Matrix::Scalar);
				GLuint glType = (GLuint)nanogui::detail::type_traits<typename Matrix::Scalar>::type;
				bool integral = (bool)nanogui::detail::type_traits<typename Matrix::Scalar>::integral;

				uploadData((uint32_t)M.size(), (int)M.rows(), compSize,
					glType, integral, (const uint8_t *)M.data());

				return *this;
			}

			/// Download the data from the vertex buffer object into an Eigen matrix
			template <typename Matrix>
			void downloadData(Matrix &M)
			{
				uint32_t compSize = sizeof(typename Matrix::Scalar);
				GLuint glType = (GLuint)nanogui::detail::type_traits<typename Matrix::Scalar>::type;

				M.resize(dim, size / dim);

				downloadData(M.size(), M.rows(), compSize, glType, (uint8_t *)M.data());
			}

			//Binds the vertex buffer to the provided attribute of the currently bound GLShader program.
			void bindToAttribute(const std::string& attribute);

			/// Return the size of this buffer in bytes
			size_t bufferSize() const
			{
				return size;
			}

			void uploadData(uint32_t size, const void *data);
			void uploadData(uint32_t size, int dim,
				uint32_t compSize, GLuint glType, bool integral,
				const uint8_t *data);
			void downloadData(uint32_t size, int dim,
				uint32_t compSize, GLuint glType, uint8_t *data);


		protected:
			GLBufferType type;

			GLuint id;
			GLuint glType;
			GLuint dim;
			GLuint compSize;
			GLuint size;
			bool integral;
		};
	}
}