#version 330

uniform mat4 mvp;
uniform float offset;

in vec3 position;
in vec3 normal;

void main()
{
	gl_Position = mvp * vec4(position + normal * offset, 1.0);
}
