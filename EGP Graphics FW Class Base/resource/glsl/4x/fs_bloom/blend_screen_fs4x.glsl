/*
	Blend
	By Dan Buckstein
	Fragment shader that blends 4 textures using the "screen" filter.
	
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
uniform sampler2D img1;
uniform sampler2D img2;
uniform sampler2D img3;


// ****
// target
layout (location = 0) out vec4 fragColor;


// shader function
void main()
{
	// ****
	// output: screen four images together
	vec4 imgSample0 = texture(img, passTexcoord);
	vec4 imgSample1 = texture(img1, passTexcoord);
	vec4 imgSample2 = texture(img2, passTexcoord);
	vec4 imgSample3 = texture(img3, passTexcoord);

	fragColor = 1.0 - (1.0 - imgSample0)*(1.0 - imgSample1)*(1.0 - imgSample2)*(1.0 - imgSample3);
}