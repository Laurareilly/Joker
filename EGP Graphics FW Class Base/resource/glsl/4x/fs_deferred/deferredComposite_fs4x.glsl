/*
	Deferred Lighting Composite
	By Dan Buckstein
	Fragment shader that composites precomputed lighting with surface color.
	
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
uniform sampler2D img_texcoord;
uniform sampler2D img_light_diffuse;
uniform sampler2D img_light_specular;


// ****
// target
layout (location = 0) out vec4 fragColor;


// shader function
void main()
{
	// ****
	// get object texture coordinates to sample surface textures, 
	//	and also sample results from light pre-pass
	vec4 texcoord = texture(img_texcoord, passTexcoord);

	vec4 diffuseSample = texture(tex_dm, texcoord.xy);
	vec4 specularSample = texture(tex_sm, texcoord.xy);

	vec4 diffuseShading = texture(img_light_diffuse, passTexcoord);
	vec4 specularShading = texture(img_light_specular, passTexcoord);

	fragColor = diffuseSample*diffuseShading + specularSample*specularShading;
}