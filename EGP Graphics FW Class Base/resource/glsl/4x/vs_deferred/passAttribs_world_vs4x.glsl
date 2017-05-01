/*
	Pass Attributes World
	By Dan Buckstein
	Vertex shader that passes attributes in world space.
	
	Modified by: ______________________________________________________________
*/

// version
#version 410


// ****
// attributes
layout (location = 0) in vec4 position;
layout (location = 2) in vec4 normal;
layout (location = 8) in vec4 texcoord;


// ****
// uniforms
uniform mat4 modelMat;
uniform mat4 viewprojMat;
uniform mat4 atlasMat;
uniform float normalScale;


// ****
// varyings
out vertexdata
{
	vec4 position_world;
	vec4 normal_world;
	vec4 texcoord_atlas;
} pass;


// shader function
void main()
{
	// ****
	// set proper clip position
	vec4 worldPos = modelMat * position;
	pass.position_world = worldPos;
	pass.normal_world = modelMat * vec4(normal.xyz*normalScale, 0.0);
	pass.texcoord_atlas = atlasMat * texcoord;
	//pass.texcoord_atlas = texcoord;
	gl_Position = viewprojMat * worldPos;
}