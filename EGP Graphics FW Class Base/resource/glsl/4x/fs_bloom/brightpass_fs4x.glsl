/*
	Bright Pass
	By Dan Buckstein
	Fragment shader that leaves bright pixels bright and makes dark darker.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in vec2 passTexcoord;


// ****
// uniforms
uniform sampler2D img;


// ****
// target
layout (location = 0) out vec4 fragColor;


// shader function
void main()
{
	// ****
	// output: make bright values brighter, make dark values darker
	vec4 imgSample = texture(img, passTexcoord);
	float luminance = 0.2126*imgSample.r + 0.7152*imgSample.g + 0.0722*imgSample.b;
	luminance *= luminance;	// ^2
	luminance *= luminance;	// ^4
	luminance *= luminance;	// ^8
	luminance *= luminance;	// ^16
	fragColor = imgSample*luminance;
}