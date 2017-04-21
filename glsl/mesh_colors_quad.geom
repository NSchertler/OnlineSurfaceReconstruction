#version 430 compatibility

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in TES_OUT
{
	vec4 pos;
	vec4 color;	
} vIn[];

out FRAG_IN
{
	vec4 pos;
	vec4 color;	
	vec3 n;
} vOut;

void main(void)
{
	vOut.n = normalize(cross(vIn[1].pos.xyz - vIn[0].pos.xyz, vIn[2].pos.xyz - vIn[0].pos.xyz));
	for(int i = 0; i < 3; ++i)
	{
		vOut.pos = vIn[i].pos;
		vOut.color = vIn[i].color;
		gl_Position = gl_in[i].gl_Position;
		EmitVertex();
	}
}