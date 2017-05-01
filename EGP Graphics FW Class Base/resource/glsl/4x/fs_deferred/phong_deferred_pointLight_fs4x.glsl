/*
	Phong Deferred - Point light
	By Dan Buckstein
	Fragment shader that performs Phong shading for a point light in a deferred 
		rendering pipeline.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in vec4 passPositionClip;


// ****
// uniforms
uniform sampler2D img_position;
uniform sampler2D img_normal;
uniform sampler2D img_depth;

uniform vec4 lightColor;
uniform vec4 lightPos;
uniform vec4 eyePos;


// ****
// target
layout (location = 0) out vec4 light_diffuse;
layout (location = 1) out vec4 light_specular;


float calculateAttenuation(float distSq, float maxDistSq)
{
	float atten = distSq / maxDistSq;
	return (1.0 - max(0.0, min(1.0, atten)));
}

void phong(float atten, in vec4 N, in vec4 L, in vec4 V, in vec4 lightColor, out vec4 diffuseShading, out vec4 specularShading)
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

	diffuseShading = (atten*kd)*lightColor;
	specularShading = (atten*ks)*lightColor;
	
	diffuseShading.w = kd;
	specularShading.w = ks;
}


// shader function
void main()
{
	// ****
	// since we did not generate this fragment using FSQ, 
	//	need to figure out where we are in screen space...
	vec2 screenspace = passPositionClip.xy * (0.5 / passPositionClip.w) + 0.5;
	//vec2 screenspace = gl_FragCoord.xy;

	// ****
	// output: calculate lighting for this light
	vec4 position = texture(img_position, screenspace);
	vec4 normal = normalize(texture(img_normal, screenspace));
	vec4 eyeVec = normalize(eyePos - position);
	vec4 lightVec = lightPos - position;

	float maxDistSq = lightColor.w*lightColor.w*0.95;
	float distSq = dot(lightVec, lightVec);
	float atten = calculateAttenuation(distSq, maxDistSq);	

	lightVec = normalize(lightVec);

	phong(atten, normal, lightVec, eyeVec, lightColor, light_diffuse, light_specular);
}