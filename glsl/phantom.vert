#version 330

in vec3 position;

uniform mat4 mvp;

void main() 
{
	vec4 p = vec4(vec4(position, 1));	
	gl_Position = mvp * p;
}
