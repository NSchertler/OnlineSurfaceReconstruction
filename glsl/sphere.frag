#version 330

out vec4 result;

in fData
{
	vec4 toPixel;
	vec4 cam;
	float radius;
} frag;

uniform mat4 p;
uniform vec4 color;
uniform bool gradient;

float rand(vec2 co)
{
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

void main(void)
{
	vec3 v = frag.toPixel.xyz - frag.cam.xyz;
	vec3 e = frag.cam.xyz;
	float ev = dot(e, v);
	float vv = dot(v, v);
	float ee = dot(e, e);
	float rr = frag.radius * frag.radius;

	float radicand = ev * ev - vv * (ee - rr);
	if(radicand < 0)
		discard;
	
	float rt = sqrt(radicand);
	

	float lambda = max(0, (-ev - rt) / vv);
	float lambda2 = (-ev + rt) / vv;
	if(lambda2 < lambda)
		discard;

	float inSphereLength = (lambda2 - lambda) / (2 * frag.radius) * sqrt(vv);

	vec3 hit = lambda * v;

	vec4 proj = p * vec4(hit, 1);
	gl_FragDepth = ((gl_DepthRange.diff * proj.z / proj.w) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;

	float value = inSphereLength * inSphereLength + 0.05 * rand(gl_FragCoord.xy);
	if(!gradient)
		value = 1;

	result = vec4(value * color.rgb, color.a);
}