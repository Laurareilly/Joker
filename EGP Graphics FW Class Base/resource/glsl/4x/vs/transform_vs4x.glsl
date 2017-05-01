/*
	Transform
	By Dan Buckstein
	Vertex shader that performs standard MVP transform.
	
	Modified by: ______________________________________________________________
*/

#version 410


layout (location = 0) in vec4 position;

uniform mat4 mvp;

void main()
{
	// set clip position 'gl_Position'
	gl_Position = mvp * position;
}