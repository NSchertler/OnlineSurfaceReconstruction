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

layout(points) in;
layout(line_strip, max_vertices = 8) out;

uniform mat4 mvp;
uniform float scale;
uniform float offset;
uniform int rosy;

in vData {
	vec3 normal;
	vec3 tangent;
} vertices[];

out fData {
	vec3 color;
} frag;

vec3 rotate60(vec3 d, vec3 n) {
	return 0.866025403784439 * cross(n, d) + 0.5 * (d + n * dot(n, d));
}

void main() {
	vec3 n = vertices[0].normal;
	vec3 p = gl_in[0].gl_Position.xyz + offset*n;
	vec3 q = vertices[0].tangent * scale;

	frag.color = vec3(1.0);
	gl_Position = mvp * vec4(p, 1.0);
	EmitVertex();
	gl_Position = mvp * vec4(p + q, 1.0);
	EmitVertex();
	EndPrimitive();
	frag.color = vec3(0.0);
	gl_Position = mvp * vec4(p, 1.0);
	EmitVertex();
	gl_Position = mvp * vec4(p - q, 1.0);
	EmitVertex();
	EndPrimitive();

	if (rosy == 4) {
		vec3 t = cross(n, q);
		gl_Position = mvp * vec4(p - t, 1.0);
		EmitVertex();
		gl_Position = mvp * vec4(p + t, 1.0);
		EmitVertex();
		EndPrimitive();
	} else if (rosy == 6) {
		vec3 t1 = rotate60(q, n);
		vec3 t2 = rotate60(q, -n);
		gl_Position = mvp * vec4(p - t1, 1.0);
		EmitVertex();
		gl_Position = mvp * vec4(p + t1, 1.0);
		EmitVertex();
		EndPrimitive();
		gl_Position = mvp * vec4(p - t2, 1.0);
		EmitVertex();
		gl_Position = mvp * vec4(p + t2, 1.0);
		EmitVertex();
		EndPrimitive();
	}
}
