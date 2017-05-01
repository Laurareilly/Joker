/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	GLSL shader and program utilities by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_SHADERPROGRAM_H
#define __EGPFW_SHADERPROGRAM_H


#include "egpfw/egpfw/utils/egpfwShaderProgramUtils.h"


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// newly-defined functions
// see utils version for descriptions of functions
// return and argument types are borrowed from the utils version

	// shader functions
	egpShader egpfwCreateShaderFromSource(const egpShaderType type, const char *source);
	egpShader egpfwCreateShaderFromSources(const egpShaderType type, const char **sources, unsigned int numSources);
	int egpfwReleaseShader(egpShader *shader);

	// program functions
	egpProgram egpfwCreateProgram();
	int egpfwAttachShaderToProgram(egpProgram *program, const egpShader *shader);
	int egpfwDetachShaderFromProgram(egpProgram *program, const egpShaderType type);
	int egpfwLinkProgram(egpProgram *program);
	int egpfwValidateProgram(egpProgram *program);
	void egpfwActivateProgram(const egpProgram *program);
	int egpfwReleaseProgram(egpProgram *program);
	int egpfwGetUniformLocation(const egpProgram *program, const char *uniformName);
	void egpfwSendUniformInt(int location, const egpUniformIntType type, const unsigned int count, const int *values);
	void egpfwSendUniformFloat(int location, const egpUniformFloatType type, const unsigned int count, const float *values);
	void egpfwSendUniformFloatMatrix(int location, const egpUniformFloatMatrixType type, const unsigned int count, const int transpose, const float *values);
	
		
//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_SHADERPROGRAM_H