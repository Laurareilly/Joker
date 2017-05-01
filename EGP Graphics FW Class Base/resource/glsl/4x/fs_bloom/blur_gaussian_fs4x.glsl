/*
	Gaussian Blur (1D)
	By Dan Buckstein
	Fragment shader that performs Gaussian blur along a single arbitrary axis.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// varyings
in vec2 passTexcoord;


// ****
// uniforms
uniform vec2 pixelSizeInv;
uniform sampler2D img;


// ****
// target
layout (location = 0) out vec4 fragColor;


// Gaussian blurring, using a 1D kernel along the provided axis 
// if using Pascal's triangle for the kernel, odd rows should be used: 
//	2^0:	1
//	2^1:	1	1
//	2^2:	1	2	1
//	2^3:	1	3	3	1
//	2^4:	1	4	6	4	1
//	2^5:	1	5	10	10	5	1
//	2^6:	1	6	15	20	15	6	1
//	2^7:	1	7	21	35	35	21	7	1
//	2^8:	1	8	28	56	70	56	28	8	1
//	2^9:	1	9	36	84	126	126	84	36	9	1
//	2^10:	1	10	45	120	210	252	210	120	45	10	1
vec4 Gaussian8(in vec2 center, in vec2 axis, in sampler2D image)
{
	vec4 result = texture(image, center) * 70.0;
	vec2 axis_n = -axis;
	vec2 samplingCoord = center;	// sampling positive direction
	vec2 samplingCoord_n = center;	// sampling negative direction
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 56.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 28.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 8.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n));
	return result / 256.0;
}
vec4 Gaussian10(in vec2 center, in vec2 axis, in sampler2D image)
{
	vec4 result = texture(image, center) * 252.0;
	vec2 axis_n = -axis;
	vec2 samplingCoord = center;	// sampling positive direction
	vec2 samplingCoord_n = center;	// sampling negative direction
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 210.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 120.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 45.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n)) * 10.0;
	result += (texture(image, samplingCoord += axis) + texture(image, samplingCoord_n += axis_n));
	return result / 1024.0;
}


// shader function
void main()
{
	// ****
	// output: Gaussian blur on an arbitrary axis
	fragColor = Gaussian10(passTexcoord, pixelSizeInv, img);
	
	//fragColor = vec4(passTexcoord.xy, 0, 0);
	//fragColor = vec4(pixelSizeInv.xy, 0, 1) * 100;

	//fragColor = texture(img, passTexcoord);
}