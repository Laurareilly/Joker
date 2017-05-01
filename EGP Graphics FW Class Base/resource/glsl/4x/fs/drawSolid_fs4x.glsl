/*
	Draw Solid Color FS
	By Dan Buckstein
	Fragment shader that outputs a solid color copied from uniform.
*/

#version 410

uniform vec4 color;

out vec4 fragColor;

void main()
{
	fragColor = color;
}