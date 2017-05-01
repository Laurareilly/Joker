// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwShaderProgram.h"


// OpenGL
#ifdef _WIN32
#include "GL/glew.h"
#else	// !_WIN32
#include <OpenGL/gl3.h>
#endif	// _WIN32


#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// shader type array
const unsigned int egpfwShaderType[] =
{
	0,
	GL_VERTEX_SHADER,
	GL_TESS_CONTROL_SHADER,
	GL_TESS_EVALUATION_SHADER,
	GL_GEOMETRY_SHADER,
	GL_FRAGMENT_SHADER,
};


//-----------------------------------------------------------------------------
// shader functions

egpShader egpfwCreateShaderFromSource(const egpShaderType type, const char *source)
{
	// this function is complete!
	return egpfwCreateShaderFromSources(type, &source, 1);
}

egpShader egpfwCreateShaderFromSources(const egpShaderType type, const char **sources, unsigned int numSources)
{
	egpShader ret = { 0 };
	//...
	return ret;
}

int egpfwReleaseShader(egpShader *shader)
{
	//...
	return 0;
}


//-----------------------------------------------------------------------------
// program functions

egpProgram egpfwCreateProgram()
{
	egpProgram ret = { 0 };
	//...
	return ret;
}

int egpfwAttachShaderToProgram(egpProgram *program, const egpShader *shader)
{
	//...
	return 0;
}

int egpfwDetachShaderFromProgram(egpProgram *program, const egpShaderType type)
{
	//...
	return 0;
}

int egpfwLinkProgram(egpProgram *program)
{
	//...
	return 0;
}

int egpfwValidateProgram(egpProgram *program)
{
	//...
	return 0;
}

void egpfwActivateProgram(const egpProgram *program)
{
	//...
}

int egpfwReleaseProgram(egpProgram *program)
{
	//...
	return 0;
}

int egpfwGetUniformLocation(const egpProgram *program, const char *uniformName)
{
	//...
	return -1;
}

void egpfwSendUniformInt(int location, const egpUniformIntType type, const unsigned int count, const int *values)
{
	//...
}

void egpfwSendUniformFloat(int location, const egpUniformFloatType type, const unsigned int count, const float *values)
{
	//...
}

void egpfwSendUniformFloatMatrix(int location, const egpUniformFloatMatrixType type, const unsigned int count, const int transpose, const float *values)
{
	//...
}


//-----------------------------------------------------------------------------
