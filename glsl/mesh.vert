#version 330

in vec3 position;
in vec3 normal;
in vec3 color;

out FRAG_IN
{
	vec4 pos;
	vec4 color;	
	vec3 n;
} vertex;

uniform mat4 mv;
uniform mat4 mvp;

void main() 
{
	vec4 p = vec4(vec4(position, 1));
	vertex.pos = mv * p;
	vertex.n = (mv * vec4(normal, 0)).xyz;	
	vertex.color = vec4(color, 1);
	gl_Position = mvp * p;
}
