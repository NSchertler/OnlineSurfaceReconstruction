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

#include "osr/gui/GLBuffer.h"
#include "osr/gui/GLShader.h"
#include <iostream>

using namespace osr::gui;

const GLuint BufferTargets[] = { GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER, GL_SHADER_STORAGE_BUFFER };

GLBuffer::GLBuffer(GLBufferType type)
	: id(0), type(type)
{}

GLBuffer::~GLBuffer()
{
	if (id != 0)
		glDeleteBuffers(1, &id);
}

void GLBuffer::bind()
{
	if (id == 0)
		glGenBuffers(1, &id);

	glBindBuffer(BufferTargets[type], id);	
}

void GLBuffer::bindBufferBase(unsigned int base) const
{
	glBindBufferBase(BufferTargets[type], base, id);

}

void GLBuffer::bindToAttribute(const std::string& attribute)
{
	assert(type == VertexBuffer);

	GLShader* prog = GLShader::currentProgram();
	auto attribID = prog->attrib(attribute);
	if (attribID >= 0)
	{
		glEnableVertexAttribArray(attribID);
		bind();
		if (integral)
			glVertexAttribIPointer(attribID, dim, glType, 0, 0);
		else
			glVertexAttribPointer(attribID, dim, glType, GL_FALSE, 0, 0);
	}
	else
		std::cout << "Warning: Attribute \"" << attribute << "\" not found in shader \"" << prog->name() << "\"." << std::endl;
}

void GLBuffer::uploadData(uint32_t size, int dim,
	uint32_t compSize, GLuint glType, bool integral,
	const uint8_t *data) 
{		
	this->glType = glType;
	this->dim = dim;
	this->compSize = compSize;
	this->size = size;
	this->integral = integral;
	
	size_t totalSize = (size_t)size * (size_t)compSize;

	bind();

	glBufferData(BufferTargets[type], totalSize, data, GL_DYNAMIC_DRAW);	
}


void GLBuffer::uploadData(uint32_t size, const void *data)
{
	this->glType = -1;
	this->dim = -1;
	this->compSize = -1;
	this->size = size;
	this->integral = false;

	bind();

	glBufferData(BufferTargets[type], size, data, GL_DYNAMIC_DRAW);
}

void GLBuffer::downloadData(uint32_t size, int /* dim */,
	uint32_t compSize, GLuint /* glType */, uint8_t *data)
{
	if(id == 0)
		throw std::runtime_error("The specified buffer has no data to download.");

	if (this->size != size || this->compSize != compSize)
		throw std::runtime_error("downloadData: size mismatch!");

	size_t totalSize = (size_t)size * (size_t)compSize;

	bind();

	glGetBufferSubData(BufferTargets[type], 0, totalSize, data);	
}