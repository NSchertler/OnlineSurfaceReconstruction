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

layout(points) in;
layout(triangle_strip, max_vertices = 4) out;

in vData
{
	vec4 pos;
	float radius;
} vertex[];

out fData
{
	vec4 toPixel;
	vec4 cam;
	float radius;
} outputV;

uniform mat4 p;

void main(void)
{
	outputV.cam = -vertex[0].pos;
	outputV.radius = vertex[0].radius;

	float d = length(vertex[0].pos);
	float o = 1.2 * (vertex[0].radius + d) / d;

	outputV.toPixel = 2 * vertex[0].radius * vec4(-o, +o, -1, 0);
	gl_Position = p * (vertex[0].pos + outputV.toPixel);
	EmitVertex();

	outputV.toPixel = 2 * vertex[0].radius * vec4(+o, +o, -1, 0);
	gl_Position = p * (vertex[0].pos + outputV.toPixel);
	EmitVertex();

	outputV.toPixel = 2 * vertex[0].radius * vec4(-o, -o, -1, 0);
	gl_Position = p * (vertex[0].pos + outputV.toPixel);
	EmitVertex();

	outputV.toPixel = 2 * vertex[0].radius * vec4(+o, -o, -1, 0);
	gl_Position = p * (vertex[0].pos + outputV.toPixel);
	EmitVertex();
}