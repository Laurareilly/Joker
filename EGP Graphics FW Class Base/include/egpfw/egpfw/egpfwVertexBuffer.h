/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Vertex buffer utilities by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_VERTEXBUFFER_H
#define __EGPFW_VERTEXBUFFER_H


#include "egpfw/egpfw/utils/egpfwVertexBufferUtils.h"


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// newly-defined functions
// see utils version for descriptions of functions
// return and argument types are borrowed from the utils version

	egpAttributeDescriptor egpfwCreateAttributeDescriptor(const egpAttributeName name, const egpAttributeType type, const void *data);
	egpVertexBufferObjectDescriptor egpfwCreateVBOInterleaved(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices);
	egpVertexBufferObjectDescriptor egpfwCreateVBOInterleavedIndexed(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out);
	egpVertexArrayObjectDescriptor egpfwCreateVAO(const egpPrimitiveType primType, egpVertexBufferObjectDescriptor *vbo, egpIndexBufferObjectDescriptor *ibo);
	egpVertexArrayObjectDescriptor egpfwCreateVAOInterleaved(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, egpIndexBufferObjectDescriptor *ibo);
	egpVertexArrayObjectDescriptor egpfwCreateVAOInterleavedIndexed(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out);
	void egpfwActivateVAO(const egpVertexArrayObjectDescriptor *vao);
	void egpfwDrawActiveVAO();
	void egpfwDrawActiveVAOPartial(const unsigned int first, const unsigned int count);
	void egpfwDrawActiveVAOInstanced(const unsigned int primCount);
	void egpfwDrawActiveVAOInstancedPartial(const unsigned int primCount, const unsigned int first, const unsigned int count);
	int egpfwReleaseVBO(egpVertexBufferObjectDescriptor *vbo);
	int egpfwReleaseIBO(egpIndexBufferObjectDescriptor *ibo);
	int egpfwReleaseVAO(egpVertexArrayObjectDescriptor *vao);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_VERTEXBUFFER_H