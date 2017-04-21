#version 330 compatibility

in vData
{
	vec4 pos;
	vec4 n;
	vec3 color;
} vertex;

out vec4 result;

const vec4 v_color = vec4(0.7, 0.7, 0.9, 1.0);

void main(void)
{
	vec3 h = normalize(-vertex.pos.xyz);
	vec3 normal = normalize(vertex.n.xyz);

	float diffuseFactor = 0.2 + 0.7 * max(0.0, abs(normalize(normal.xyz).z));
	
	//Inside faces
	if(dot(normal.xyz, vertex.pos.xyz) > 0)
		diffuseFactor *= 0.5;

	float specularFactor = pow(abs(dot(h, normal)), 75);

	result = vec4(diffuseFactor * vertex.color, 1) + specularFactor * vec4(1, 1, 1, 0);
}