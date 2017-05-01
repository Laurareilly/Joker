/*
	Pass Color
	By Dan Buckstein
	Vertex shader that passes color attribute down pipeline.
	
	Modified by: ______________________________________________________________
*/

// which version of GLSL is this shader written in
#version 410


// ****
// attributes: data read in directly from a vertex in VBO
// format for a single attribute: 
//		layout (location = <attribute index>) in <type> <name>;

layout (location = 0) in vec4 position;
layout (location = 2) in vec3 normal;


// ****
// uniforms: values that are the same for the entire primitive
// in GLSL 4.3+ you can hard-set these like attributes: 
//		layout (location = <uniform index>) uniform <type> <name>;
// ...or normally (before 4.3): 
//		uniform <type> <name>;
uniform mat4 mvp;


// ****
// varyings: data passed to the next stage in the pipeline
// in GLSL 4.x you can use structure format: 
//		out <structure name> {
//			<type> <name>;		// <- do this for each one
//		} <output name>;
// ...or one-by-one (compatible with version 3.x): 
//		out <type> <name>;		// <- do this for each one
out vec3 passNormal;


// shader entry point: function executes once per-vertex
void main()
{
	// ****
	// required in vertex processing: set clip position 'gl_Position'
	// this example: multiply object-space position (within model) by full-
	//	stack 'MVP' matrix to get clip-space position that OpenGL needs
	vec4 pos = position + vec4(normalize(normal).xyz * 0.05, 0);
	gl_Position = mvp * pos;

	// ****
	// optional step: pass data along to next stage in pipeline
	// this example: copy inbound color attribute directly to outbound varying
	passNormal = normal;
}