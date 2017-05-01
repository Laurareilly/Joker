/*
	Draw Color
	By Dan Buckstein
	Fragment shader that displays color received from previous piipeline stage.
	
	Modified by: ______________________________________________________________
*/

// version (same idea)
#version 410


// ****
// varyings: data received from previous stage in the pipeline
// same formats as vertex shader, with restrictions: 
//	-> for structure format, full structure layout must be identical
//		-> replace 'out' with 'in':		in <structure name> { ...
//	-> for one-by-one format, variable type and name must be identical
in vec4 passColor;


// uniforms: same idea for all shader types
// (none in this example)


// ****
// target: fragment shader result to be stored in framebuffer
// same format as attribute, but with 'out' instead of 'in': 
//		layout (location = <target index>) out <type> <name>;
layout (location = 0) out vec4 fragColor;


// shader entry point: function executes once per-fragment
void main()
{
	// ****
	// no required steps, but the goal is to assign a color to a target
	// this example: copy inbound color varying directly to framebuffer target
	fragColor = passColor;
}