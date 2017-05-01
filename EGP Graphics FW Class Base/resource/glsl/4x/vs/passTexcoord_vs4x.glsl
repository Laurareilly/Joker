/*
	Pass Texcoord
	By Dan Buckstein
	Vertex shader that passes texcoord attribute down pipeline.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// attributes
layout (location = 0) in vec4 position;
layout (location = 8) in vec4 texcoord;


// ****
// uniforms
uniform mat4 mvp;


// ****
// varyings
out vec2 passTexcoord;


// shader function
void main()
{
	// ****
	// set proper clip position
	gl_Position = mvp * position;

	// ****
	// pass data
	passTexcoord = texcoord.xy;
}