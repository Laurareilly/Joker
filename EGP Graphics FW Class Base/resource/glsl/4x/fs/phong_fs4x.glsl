/*
	Phong (FS)
	By Dan Buckstein
	Fragment shader that computes Phong shading.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in VertexData
{
	vec4 normal;
	vec4 lightVec;
	vec4 eyeVec;
	vec4 texcoord;
} pass;


// ****
// uniforms
uniform sampler2D tex_dm;
uniform sampler2D tex_sm;


// ****
// target
layout (location = 0) out vec4 fragColor;



const vec4 AMBIENT = vec4(0.08, 0.04, 0.02, 1.0);

// shader function
void main()
{
	// ****
	// this example: phong shading algorithm
	
	// 1. diffuse
	vec4 N = normalize(pass.normal);
	vec4 L = normalize(pass.lightVec);
	float kd = dot(N, L);

	// 2. specular
	vec4 V = normalize(pass.eyeVec);
	vec4 R = (kd+kd)*N - L;
	float ks = dot(V, R);

	// 3. clamp & exponent
	kd = max(0.0, kd);
	ks = max(0.0, ks);
	ks *= ks;
	ks *= ks;
	ks *= ks;
	ks *= ks;

	// ****
	// output: phong
	//	fragColor = vec4(kd + ks) + AMBIENT;

	// ****
	// extra: apply textures as well
	fragColor = kd*texture(tex_dm, pass.texcoord.xy) + ks*texture(tex_sm, pass.texcoord.xy) + AMBIENT;
}