#version 330
precision lowp float;

out vec4 outColor;
in vec3 color_frag;

void main() 
{
	outColor = vec4(color_frag, 1.0);
}

