/*
	Deferred Shading
	By Dan Buckstein
	Fragment shader that performs shading using geometry buffers.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in vec2 passTexcoord;


// ****
// uniforms
uniform sampler2D tex_dm;
uniform sampler2D tex_sm;
uniform sampler2D img_position;
uniform sampler2D img_normal;
uniform sampler2D img_depth;
uniform sampler2D img_texcoord;

#define NUM_LIGHTS 4
uniform vec4 lightColor[NUM_LIGHTS];
uniform vec4 lightPos[NUM_LIGHTS];
uniform vec4 eyePos;


// ****
// target
layout (location = 0) out vec4 fragColor;


float calculateAttenuation(float distSq, float maxDistSq)
{
	float atten = distSq / maxDistSq;
	return (1.0 - max(0.0, min(1.0, atten)));
}

float evaluateLight(int i, in vec4 position, out vec4 lightVec, out vec4 lightCol)
{
	float maxDistSq = lightColor[i].w*lightColor[i].w;
	lightCol = lightColor[i];
	lightVec = lightPos[i] - position;
	float distSq = dot(lightVec, lightVec);
	lightVec = normalize(lightVec);
	return calculateAttenuation(distSq, maxDistSq);
}

vec4 phong(float atten, in vec4 N, in vec4 L, in vec4 V, in vec4 lightColor, in vec4 diffuseColor, in vec4 specularColor)
{
	float kd = dot(N, L);

	vec4 R = (kd+kd)*N - L;
	float ks = dot(V, R);

	kd = max(0.0, kd);
	ks = max(0.0, ks);

	ks *= ks;
	ks *= ks;
	ks *= ks;
	ks *= ks;

	return (kd*diffuseColor + ks*specularColor)*lightColor*atten;
}


// shader function
void main()
{
	// ****
	// output: calculate lighting for each light by reading in 
	//	attribute data from g-buffers
	vec4 position = texture(img_position, passTexcoord);
	vec4 normal = normalize(texture(img_normal, passTexcoord));
	vec4 texcoord = texture(img_texcoord, passTexcoord);

	vec4 diffuseSample = texture(tex_dm, texcoord.xy);
	vec4 specularSample = texture(tex_sm, texcoord.xy);

	vec4 eyeVec = normalize(eyePos - position);
	vec4 lightVec, lightCol, lighting = vec4(0.0);
	float atten;

	atten = evaluateLight(0, position, lightVec, lightCol);
	lighting += phong(atten, normal, lightVec, eyeVec, lightCol, diffuseSample, specularSample);
	
	atten = evaluateLight(1, position, lightVec, lightCol);
	lighting += phong(atten, normal, lightVec, eyeVec, lightCol, diffuseSample, specularSample);

	atten = evaluateLight(2, position, lightVec, lightCol);
	lighting += phong(atten, normal, lightVec, eyeVec, lightCol, diffuseSample, specularSample);

	atten = evaluateLight(3, position, lightVec, lightCol);
	lighting += phong(atten, normal, lightVec, eyeVec, lightCol, diffuseSample, specularSample);

	fragColor = lighting;
}