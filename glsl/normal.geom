#version 330

layout(points) in;
layout(line_strip, max_vertices = 2) out;

uniform mat4 mvp;
uniform float scale;

in vData {
	vec3 normal;
} vertices[];

out fData {
	vec3 color;
} frag;

void main() {
	vec3 n = vertices[0].normal;
	vec3 p = gl_in[0].gl_Position.xyz;

	frag.color = vec3(1, 0, 0);
	gl_Position = mvp * vec4(p, 1);
	EmitVertex();

	frag.color = vec3(1, 1, 1);
	gl_Position = mvp * vec4(p + scale * n, 1);
	EmitVertex();
}
