/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	GLSL shader and program utilities by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_SHADERPROGRAM_H
#define __EGPFW_UTILS_SHADERPROGRAM_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// enumerators

	// shader types
	enum egpShaderType
	{
		EGP_SHADER_INVALID, 
		EGP_SHADER_VERTEX, 
		EGP_SHADER_TESS_CTRL, 
		EGP_SHADER_TESS_EVAL, 
		EGP_SHADER_GEOMETRY, 
		EGP_SHADER_FRAGMENT, 
	
		// there is one more type... discover it!
	};

	// uniform types (int-based)
	enum egpUniformIntType
	{
		UNIF_INT,		// single integer
		UNIF_IVEC2,		// 2D integer vector
		UNIF_IVEC3,		// 3D integer vector
		UNIF_IVEC4,		// 4D integer vector
	};

	// uniform types (float-based)
	enum egpUniformFloatType
	{
		UNIF_FLOAT,		// single float
		UNIF_VEC2,		// 2D float vector
		UNIF_VEC3,		// 3D float vector
		UNIF_VEC4,		// 4D float vector
	};

	// uniform types (float-based matrix)
	enum egpUniformFloatMatrixType
	{
		UNIF_MAT2, 		// 2x2 float matrix
		UNIF_MAT3, 		// 3x3 float matrix
		UNIF_MAT4, 		// 4x4 float matrix
		UNIF_MAT3x2,	// 3 col x 2 row matrix
		UNIF_MAT4x2, 	// 4 col x 2 row matrix
		UNIF_MAT4x3, 	// 4 col x 3 row matrix
	};

#ifndef __cplusplus
	typedef enum egpShaderType egpShaderType;
	typedef enum egpUniformIntType egpUniformIntType;
	typedef enum egpUniformFloatType egpUniformFloatType;
	typedef enum egpUniformFloatMatrixType egpUniformFloatMatrixType;
#endif	// __cplusplus

	
//-----------------------------------------------------------------------------
// data structures

	// structure for GLSL shader
	struct egpShader
	{
		// initialize with a type, pass to 'create' function
		// it will handle the rest
		unsigned int glhandle;
		int compiled;
		egpShaderType type;
	};

#ifndef __cplusplus
	typedef struct egpShader egpShader;
#endif	// __cplusplus


	// structure for GLSL program
	struct egpProgram
	{
		unsigned int glhandle;
		int linked, validated;
		int hasShaderType[5];
	};

#ifndef __cplusplus
	typedef struct egpProgram egpProgram;
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// shader functions

	// create GLSL shader
	// pass in a shader type and raw source string(s)
	// if successful, the output object will have a handle 
	//	and the compiled flag will be set to 1
	// if failed, an error log will be printed to the console
	// 'type' param cannot be 'invalid' (zero)
	// 'source(s)' param(s) cannot be null
	// 'numSources' param cannot be zero
	egpShader egpCreateShaderFromSource(const egpShaderType type, const char *source);
	egpShader egpCreateShaderFromSources(const egpShaderType type, const char **sources, unsigned int numSources);

	// free up GLSL shader
	// returns 1 if success, 0 if failed
	// if successful, the input object will have all zero values
	// 'shader' param cannot be null
	int egpReleaseShader(egpShader *shader);


//-----------------------------------------------------------------------------
// program functions

	// create program
	// if successful, the output object will have a handle 
	//	and other flags all set to zero
	egpProgram egpCreateProgram();

	// attach an existing shader to a program object
	// returns 1 if success, 0 if failed
	// fails if that slot is already occupied
	// call next function (detach) to remove an attached shader
	// params cannot be null
	int egpAttachShaderToProgram(egpProgram *program, const egpShader *shader);

	// detach shader of specified type from existing program
	// returns 1 if success, 0 if failed (that shader type was 
	//	not previously attached to program)
	// 'program' param cannot be null
	int egpDetachShaderFromProgram(egpProgram *program, const egpShaderType type);

	// mandatory step: link program
	// call when all shaders have been attached
	// returns 1 if success, 0 if failed (will print log)
	// if successful, the input object's link flag will be 1
	// 'program' param cannot be null
	int egpLinkProgram(egpProgram *program);

	// optional step: validate program against current context
	// call after linking
	// returns 1 if success, 0 if failed
	// 'program' param cannot be null
	int egpValidateProgram(egpProgram *program);

	// use a program
	// pass in a valid program object to use it
	// pass null to disable program
	// NOTE: only ONE program is ever used at a time!
	void egpActivateProgram(const egpProgram *program);

	// free up a program
	// returns 1 if success, 0 if failed
	// if successful, the input objecty will have all zero values
	// 'program' param cannot be null
	int egpReleaseProgram(egpProgram *program);

	// query a uniform location
	// returns the handle to the variable within the program
	// params cannot be null
	int egpGetUniformLocation(const egpProgram *program, const char *uniformName);

	// send uniform values to whatever program is currently active
	// 'count' param will be defaulted to 1 if it is zero
	// 'values' param cannot be null
	void egpSendUniformInt(int location, const egpUniformIntType type, const unsigned int count, const int *values);
	void egpSendUniformFloat(int location, const egpUniformFloatType type, const unsigned int count, const float *values);
	void egpSendUniformFloatMatrix(int location, const egpUniformFloatMatrixType type, const unsigned int count, const int transpose, const float *values);
	
		
//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_SHADERPROGRAM_H