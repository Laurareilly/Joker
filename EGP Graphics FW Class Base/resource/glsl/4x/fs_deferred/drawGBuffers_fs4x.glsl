/*
	Draw G-Buffers
	By Dan Buckstein
	Fragment shader that outputs incoming geometric attributes as color.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in vertexdata
{
	vec4 position_world;
	vec4 normal_world;
	vec4 texcoord_atlas;
} pass;


// ****
// target
layout (location = 0) out vec4 gbuffer_position;
layout (location = 1) out vec4 gbuffer_normal;
layout (location = 2) out vec4 gbuffer_texcoord;


// debugging
uniform sampler2D tex_dm;


// shader function
void main()
{
	// ****
	// copy inbound values to their respective g-buffer
	gbuffer_position = pass.position_world;
	gbuffer_texcoord = pass.texcoord_atlas;
	gbuffer_normal = pass.normal_world;

	// debugging
//	gbuffer_normal = 0.5 + 0.5*gbuffer_normal;	// works
//	gbuffer_texcoord = texture(tex_dm, gbuffer_texcoord.xy);	// works
}