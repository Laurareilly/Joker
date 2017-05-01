/*
	Pass Texcoord, Pass-Thru Position
	By Dan Buckstein
	Vertex shader that passes texcoord. Does not transform vertex position.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// attributes
layout (location = 0) in vec4 position;
layout (location = 8) in vec4 texcoord;


// ****
// varyings
out vec2 passTexcoord;


// shader function
void main()
{
	// ****
	// copy position attribute directly to GL's clip position
	gl_Position = position;

	// ****
	// pass data
	passTexcoord = texcoord.xy;
}