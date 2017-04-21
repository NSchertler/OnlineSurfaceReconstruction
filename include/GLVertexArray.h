#pragma once

#include <glad/glad.h>

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