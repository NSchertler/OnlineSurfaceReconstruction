#version 330

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
	vec4 p = vec4(position, 1);
	vertex.pos = mv * p;
	vertex.n = (mv * vec4(normal, 0)).xyz;	
	vertex.color = vec4(color, 1);
	gl_Position = mvp * p;
}
