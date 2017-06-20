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

#include "GLShader.h"

#include <iostream>
#include <fstream>

using namespace osr::gui;

GLShader* GLShader::_currentProgram = nullptr;

GLShader* GLShader::currentProgram()
{
	return _currentProgram;
}

GLShader::~GLShader()
{
	glDeleteProgram(mProgramShader); mProgramShader = 0;
	glDeleteShader(mVertexShader);   mVertexShader = 0;
	glDeleteShader(mTessellationControlShader);   mTessellationControlShader = 0;
	glDeleteShader(mTessellationEvalShader);   mTessellationEvalShader = 0;
	glDeleteShader(mFragmentShader); mFragmentShader = 0;
	glDeleteShader(mGeometryShader); mGeometryShader = 0;
}

static GLuint createShader_helper(GLint type, const std::string &name,
	const std::string &defines,
	std::string shader_string)
{
	if (shader_string.empty())
		return (GLuint)0;

	if (!defines.empty()) 
	{
		if (shader_string.length() > 8 && shader_string.substr(0, 8) == "#version") 
		{
			std::istringstream iss(shader_string);
			std::ostringstream oss;
			std::string line;
			std::getline(iss, line);
			oss << line << std::endl;
			oss << defines;
			while (std::getline(iss, line))
				oss << line << std::endl;
			shader_string = oss.str();
		}
		else
		{
			shader_string = defines + shader_string;
		}
	}

	GLuint id = glCreateShader(type);
	const char *shader_string_const = shader_string.c_str();
	glShaderSource(id, 1, &shader_string_const, nullptr);
	glCompileShader(id);

	GLint status;
	glGetShaderiv(id, GL_COMPILE_STATUS, &status);

	if (status != GL_TRUE) 
	{
		char buffer[512];
		std::cerr << "Error while compiling ";
		if (type == GL_VERTEX_SHADER)
			std::cerr << "vertex shader";
		else if (type == GL_FRAGMENT_SHADER)
			std::cerr << "fragment shader";
		else if (type == GL_GEOMETRY_SHADER)
			std::cerr << "geometry shader";
		else if (type == GL_TESS_CONTROL_SHADER)
			std::cerr << "tessellation control shader";
		else if (type == GL_TESS_EVALUATION_SHADER)
			std::cerr << "tessellation evaluation shader";
		std::cerr << " \"" << name << "\":" << std::endl;
		std::cerr << shader_string << std::endl << std::endl;
		glGetShaderInfoLog(id, 512, nullptr, buffer);
		std::cerr << "Error: " << std::endl << buffer << std::endl;
		throw std::runtime_error("Shader compilation failed!");
	}

	return id;
}

bool GLShader::initFromFiles (const std::string &name,
	const std::string &vertex_fname,
	const std::string &fragment_fname,
	const std::string &geometry_fname) 
{
	auto file_to_string = [](const std::string &filename) -> std::string 
	{
		if (filename.empty())
			return "";
		std::ifstream t(filename);
		return std::string((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());
	};

	return init(name,
		file_to_string(vertex_fname),
		file_to_string(fragment_fname),
		file_to_string(geometry_fname));
}

bool GLShader::init(const std::string &name,
	const std::string &vertex_str,
	const std::string &fragment_str,
	const std::string &geometry_str)
{
	std::string defines;
	for (auto def : mDefinitions)
		defines += std::string("#define ") + def.first + std::string(" ") + def.second + "\n";

	mName = name;
	mVertexShader =
		createShader_helper(GL_VERTEX_SHADER, name, defines, vertex_str);
	mGeometryShader =
		createShader_helper(GL_GEOMETRY_SHADER, name, defines, geometry_str);
	mFragmentShader =
		createShader_helper(GL_FRAGMENT_SHADER, name, defines, fragment_str);

	if (!mVertexShader || !mFragmentShader)
		return false;
	if (!geometry_str.empty() && !mGeometryShader)
		return false;

	mProgramShader = glCreateProgram();

	glAttachShader(mProgramShader, mVertexShader);
	glAttachShader(mProgramShader, mFragmentShader);

	if (mGeometryShader)
		glAttachShader(mProgramShader, mGeometryShader);

	glLinkProgram(mProgramShader);

	GLint status;
	glGetProgramiv(mProgramShader, GL_LINK_STATUS, &status);

	if (status != GL_TRUE) {
		char buffer[512];
		glGetProgramInfoLog(mProgramShader, 512, nullptr, buffer);
		std::cerr << "Linker error (" << mName << "): " << std::endl << buffer << std::endl;
		mProgramShader = 0;
		throw std::runtime_error("Shader linking failed!");
	}

	return true;
}

bool GLShader::initWithTessellation(const std::string &name,
	const std::string &vertex_str,
	const std::string &tessellation_control_str,
	const std::string &tessellation_eval_str,
	const std::string &fragment_str,
	const std::string &geometry_str)
{
	std::string defines;
	for (auto def : mDefinitions)
		defines += std::string("#define ") + def.first + std::string(" ") + def.second + "\n";

	mName = name;
	mVertexShader =
		createShader_helper(GL_VERTEX_SHADER, name, defines, vertex_str);
	mTessellationControlShader =
		createShader_helper(GL_TESS_CONTROL_SHADER, name, defines, tessellation_control_str);
	mTessellationEvalShader =
		createShader_helper(GL_TESS_EVALUATION_SHADER, name, defines, tessellation_eval_str);
	mGeometryShader =
		createShader_helper(GL_GEOMETRY_SHADER, name, defines, geometry_str);
	mFragmentShader =
		createShader_helper(GL_FRAGMENT_SHADER, name, defines, fragment_str);

	if (!mVertexShader || !mFragmentShader || !mTessellationControlShader || !mTessellationEvalShader)
		return false;
	if (!geometry_str.empty() && !mGeometryShader)
		return false;

	mProgramShader = glCreateProgram();

	glAttachShader(mProgramShader, mVertexShader);
	glAttachShader(mProgramShader, mTessellationControlShader);
	glAttachShader(mProgramShader, mTessellationEvalShader);
	glAttachShader(mProgramShader, mFragmentShader);

	if (mGeometryShader)
		glAttachShader(mProgramShader, mGeometryShader);

	glLinkProgram(mProgramShader);

	GLint status;
	glGetProgramiv(mProgramShader, GL_LINK_STATUS, &status);

	if (status != GL_TRUE) 
	{
		char buffer[512];
		glGetProgramInfoLog(mProgramShader, 512, nullptr, buffer);
		std::cerr << "Linker error (" << mName << "): " << std::endl << buffer << std::endl;
		mProgramShader = 0;
		throw std::runtime_error("Shader linking failed!");
	}

	return true;
}

void GLShader::bind()
{
	glUseProgram(mProgramShader);
	_currentProgram = this;
}

GLint GLShader::attrib(const std::string &name, bool warn) const 
{
	GLint id = glGetAttribLocation(mProgramShader, name.c_str());
	if (id == -1 && warn)
		std::cerr << mName << ": warning: did not find attrib " << name << std::endl;
	return id;
}

GLint GLShader::uniform(const std::string &name, bool warn) const 
{
	GLint id = glGetUniformLocation(mProgramShader, name.c_str());
	if (id == -1 && warn)
		std::cerr << mName << ": warning: did not find uniform " << name << std::endl;
	return id;
}