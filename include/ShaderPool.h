#pragma once

#include "GLShader.h"

//Singleton. Holds all shaders during the runtime of the application.
class ShaderPool
{
private:
	static ShaderPool* _instance;
	ShaderPool();

public:
	static ShaderPool* Instance();
	void CompileAll();
	
	GLShader ObjectShader;
	GLShader MeshColorsTriShader;
	GLShader MeshColorsQuadShader;
	GLShader TessellatedEdgesShader;
	GLShader NormalShader;
	GLShader OrientationFieldShader;
	GLShader PositionFieldShader;
	GLShader AdjacencyShader;
	GLShader SphereShader;
	GLShader PhantomShader;
};