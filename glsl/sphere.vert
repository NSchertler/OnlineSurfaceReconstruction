#version 330

in vec4 positionRadius;

out vData
{
	vec4 pos;
	float radius;
} vertex;

uniform mat4 mv;

void main() 
{
	vec4 p = vec4(positionRadius.xyz, 1);
	vertex.pos = mv * p;
	vertex.radius = positionRadius.w * length(mv[0].xyz);
}
