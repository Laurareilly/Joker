/*
	Phong - Pass Alt Clip Position
	By Dan Buckstein
	Vertex shader that performs vertex requirements for Phong shading, but also
		passes clip position of light for shadow mapping/projective texturing.
	
	Modified by: ______________________________________________________________
*/

#version 410


// ****
// attributes: 
layout (location = 0) in vec4 position;
layout (location = 2) in vec4 normal;
layout (location = 8) in vec4 texcoord;


// ****
// uniforms: 
uniform mat4 mvp;
uniform mat4 mvp_projector;
uniform mat4 atlasMat;

uniform vec4 lightPos;
uniform vec4 eyePos;
uniform float normalScale;


// ****
// varyings: 
out vertexdata
{
	vec4 position_clip_alt;
	vec4 normal;
	vec4 texcoord;
	vec4 lightVec;
	vec4 eyeVec;
} pass;


void main()
{
	// set clip position 'gl_Position'
	gl_Position = mvp * position;
	pass.position_clip_alt = mvp_projector * position;

	pass.normal = vec4(normal.xyz * normalScale, 0.0);
	pass.texcoord = atlasMat * texcoord;

	pass.lightVec = lightPos - position;
	pass.eyeVec = eyePos - position;
}