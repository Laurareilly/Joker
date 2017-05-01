// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwVertexBuffer.h"


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


#define BUFFER_OFFSET(n) ((char *)(0) + n)


// pointer to active VAO
const egpVertexArrayObjectDescriptor *egpfw_vao_active = 0;

// data descriptors aligning with enumerated values in header
const unsigned int egpfwAttribInternalElems[] =
{
	0, 1, 2, 3, 4, 1, 2, 3, 4
};

const unsigned int egpfwAttribInternalSize[] =
{
	0, 4, 8, 12, 16, 4, 8, 12, 16
};

const unsigned int egpfwAttribInternalType[] =
{
	0, GL_INT, GL_INT, GL_INT, GL_INT, GL_FLOAT, GL_FLOAT, GL_FLOAT, GL_FLOAT
};

const unsigned int egpfwIndexInternalSize[] =
{
	0, 1, 1, 2, 2, 4, 4
};

const unsigned int egpfwIndexInternalType[] =
{
	0, GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT, GL_INT, GL_UNSIGNED_INT
};

const unsigned int egpfwPrimitive[] =
{
	GL_POINTS, GL_LINES, GL_LINE_STRIP, GL_LINE_LOOP, GL_TRIANGLES, GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN
};


//-----------------------------------------------------------------------------
// functions

egpAttributeDescriptor egpfwCreateAttributeDescriptor(const egpAttributeName name, const egpAttributeType type, const void *data)
{
	egpAttributeDescriptor attr = { 0 };
	//...
	return attr;
}

egpVertexBufferObjectDescriptor egpfwCreateVBOInterleaved(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices)
{
	egpVertexBufferObjectDescriptor vbo = { 0 };
	//...
	return vbo;
}

egpVertexBufferObjectDescriptor egpfwCreateVBOInterleavedIndexed(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out)
{
	egpVertexBufferObjectDescriptor vbo = egpfwCreateVBOInterleaved(attribs, numAttribs, numVertices);
//	egpIndexBufferObjectDescriptor ibo = { 0 };
	//...
	return vbo;
}

egpVertexArrayObjectDescriptor egpfwCreateVAO(const egpPrimitiveType primType, egpVertexBufferObjectDescriptor *vbo, egpIndexBufferObjectDescriptor *ibo)
{
	egpVertexArrayObjectDescriptor vao = { 0 };
	//...
	return vao;
}

egpVertexArrayObjectDescriptor egpfwCreateVAOInterleaved(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, egpIndexBufferObjectDescriptor *ibo)
{
	// this function is complete!
	egpVertexArrayObjectDescriptor vao;
	if (vbo_out)
	{
		*vbo_out = egpfwCreateVBOInterleaved(attribs, numAttribs, numVertices);
		vao = egpfwCreateVAO(primType, vbo_out, ibo);
	}
	else
		printf("\n VAO creation failed! No VBO-out pointer passed to function.");

	// done
	return vao;
}

egpVertexArrayObjectDescriptor egpfwCreateVAOInterleavedIndexed(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out)
{
	// this function is complete!
	egpVertexArrayObjectDescriptor vao;
	if (vbo_out)
	{
		*vbo_out = egpfwCreateVBOInterleavedIndexed(attribs, numAttribs, numVertices, indexType, numIndices, indexData, ibo_out);
		vao = egpfwCreateVAO(primType, vbo_out, ibo_out);
	}
	else
		printf("\n VAO creation failed! No VBO-out pointer passed to function.");

	// done
	return vao;
}

void egpfwActivateVAO(const egpVertexArrayObjectDescriptor *vao)
{
	//...
}

void egpfwDrawActiveVAO()
{
	//...
}

void egpfwDrawActiveVAOPartial(const unsigned int first, const unsigned int count)
{
	//...
}

void egpfwDrawActiveVAOInstanced(const unsigned int primCount)
{
	//...
}

void egpfwDrawActiveVAOInstancedPartial(const unsigned int primCount, const unsigned int first, const unsigned int count)
{
	//...
}

int egpfwReleaseVBO(egpVertexBufferObjectDescriptor *vbo)
{
	//...
	return 0;
}

int egpfwReleaseIBO(egpIndexBufferObjectDescriptor *ibo)
{
	//...
	return 0;
}

int egpfwReleaseVAO(egpVertexArrayObjectDescriptor *vao)
{
	//...
	return 0;
}


//-----------------------------------------------------------------------------
