#version 330

/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in GEOM_IN
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