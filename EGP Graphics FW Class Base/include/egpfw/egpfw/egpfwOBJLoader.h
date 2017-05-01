/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Wavefront OBJ file loader by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_OBJLOADER_H
#define __EGPFW_OBJLOADER_H


#include "egpfw/egpfw/egpfwVertexBuffer.h"


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// enumerators

	// normal calculation mode
	enum egpMeshNormalMode
	{
		NORMAL_LOAD,					// load normals from file
		NORMAL_COMPUTE_FACE,			// ignore normals from file and compute per-face
		NORMAL_COMPUTE_VERTEX,			// ignore normals from file and compute per-vertex
		NORMAL_COMPUTE_TANGENT_FACE,	// compute full tangent basis per-face (texcoords must exist)
		NORMAL_COMPUTE_TANGENT_VERTEX,	// compute full tangent basis per-vertex (texcoords must exist)
	};

#ifndef __cplusplus
	typedef enum egpMeshNormalMode egpMeshNormalMode;
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// data structures

#ifndef __cplusplus
	typedef struct egpTriOBJDescriptor egpTriOBJDescriptor;
#endif	// __cplusplus

	// trianglulated OBJ descriptor
	// NOTE: this is a SHARED RESOURCE; do not load the same OBJ multiple times
	//	because that wastes memory; just *draw* it multiple times!
	// stores all attribute data contiguously
	struct egpTriOBJDescriptor
	{
		unsigned int attribOffset[16];
		unsigned int dataSize;
		void *data;
	};


//-----------------------------------------------------------------------------
// functions

	// load triangulated OBJ file
	// returns descriptor with fully-ordered, non-indexed attribute data
	// 'objPath' param cannot be null or an empty string
	// 'globalScale' will default to 1 if not positive
	egpTriOBJDescriptor egpfwLoadTriangleOBJ(const char *objPath, const egpMeshNormalMode normalMode, const double globalScale);

	// convert OBJ to VAO & VBO
	// returns number of active attributes if successful, 0 if failed
	// 'obj' param cannot be null and must be initialized
	// 'vao_out' and 'vbo_out' params cannot be null and must be empty
	int egpfwCreateVAOFromOBJ(const egpTriOBJDescriptor *obj, egpVertexArrayObjectDescriptor *vao_out, egpVertexBufferObjectDescriptor *vbo_out);

	// free obj data
	// returns 1 if successful, 0 if failed
	int egpfwReleaseOBJ(egpTriOBJDescriptor *obj);

	// write binary version to file for fast loading next time
	// additionally a load function which returns a new object
	// returns 1 if successful, 0 if failed
	// 'obj' param cannot be null and must be initialized
	// 'binPath' param cannot be null or an empty string
	int egpfwSaveBinaryOBJ(const egpTriOBJDescriptor *obj, const char *binPath);
	egpTriOBJDescriptor egpfwLoadBinaryOBJ(const char *binPath);

	// get attribute data from OBJ
	// 'obj' param cannot be null
	// returns null if the OBJ does not use the requested attribute
	// texcoords are 2D vectors
	// positions, normals, tangents, bitangents are 3D vectors
	// skin weights (float) and indices (int) are 4D vectors
	const void *egpfwGetOBJAttributeData(const egpTriOBJDescriptor *obj, const egpAttributeName attrib);
	unsigned int egpfwGetOBJNumVertices(const egpTriOBJDescriptor *obj);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_OBJLOADER_H