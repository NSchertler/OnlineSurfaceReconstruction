#include "GLVertexArray.h"

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
