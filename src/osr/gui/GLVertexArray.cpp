/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/gui/GLVertexArray.h"

using namespace osr::gui;

GLVertexArray::GLVertexArray()
	: vaoId(0)
{
}

GLVertexArray::~GLVertexArray()
{
	if (vaoId > 0)
		glDeleteVertexArrays(1, &vaoId);
}

void GLVertexArray::generate()
{
	if (vaoId == 0)
		glGenVertexArrays(1, &vaoId);
}

void GLVertexArray::bind() const
{
	glBindVertexArray(vaoId);
}

void GLVertexArray::unbind() const
{
	glBindVertexArray(0);
}

bool GLVertexArray::valid() const
{
	return vaoId != 0;
}
