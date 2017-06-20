#version 330

/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
*/

in vec3 position;
in vec3 normal;
in vec3 tangent;

out vData {
	vec3 normal;
	vec3 tangent;
} vertex;

void main() {
	gl_Position = vec4(position, 1.0);
	vertex.normal = normal;
	vertex.tangent = tangent;
}
