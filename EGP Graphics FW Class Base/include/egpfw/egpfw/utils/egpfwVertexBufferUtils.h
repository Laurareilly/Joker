/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Vertex buffer utilities by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_VERTEXBUFFER_H
#define __EGPFW_UTILS_VERTEXBUFFER_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// enumerators

	// attribute names (what they represent in shader land)
	// these are organized in this order because they align 
	//	with the default attributes that OpenGL uses
	enum egpAttributeName
	{
		// defaults
		ATTRIB_POSITION,
		ATTRIB_BLEND_WEIGHTS,
		ATTRIB_NORMAL,
		ATTRIB_COLOR,
		ATTRIB_COLOR_SECONDARY,
		ATTRIB_FOG_COORD,
		ATTRIB_POINT_SIZE,
		ATTRIB_BLEND_INDICES,
		ATTRIB_TEXCOORD0,
		ATTRIB_TEXCOORD1,
		ATTRIB_TEXCOORD2,
		ATTRIB_TEXCOORD3,
		ATTRIB_TEXCOORD4,
		ATTRIB_TEXCOORD5,
		ATTRIB_TEXCOORD6,
		ATTRIB_TEXCOORD7,

		// alternatives
		ATTRIB_TEXCOORD = ATTRIB_TEXCOORD0,
		ATTRIB_TEXCOORD_SECONDARY,
		ATTRIB_TANGENT,
		ATTRIB_BITANGENT,
		ATTRIB_POSITION_SECONDARY,
		ATTRIB_NORMAL_SECONDARY,
		ATTRIB_TANGENT_SECONDARY,
		ATTRIB_BITANGENT_SECONDARY,
	};

	// attribute types
	enum egpAttributeType
	{
		ATTRIB_DISABLE,			// attribute is not used
		ATTRIB_INT,				// single integer
		ATTRIB_IVEC2,			// 2D integer vector
		ATTRIB_IVEC3,			// 3D integer vector
		ATTRIB_IVEC4,			// 4D integer vector
		ATTRIB_FLOAT,			// single float
		ATTRIB_VEC2,			// 2D float vector
		ATTRIB_VEC3,			// 3D float vector
		ATTRIB_VEC4,			// 4D float vector
	};

	// index types for IBO
	enum egpIndexType
	{
		INDEX_DISABLE,			// IBO not used
		INDEX_BYTE,				// byte (max 127)
		INDEX_UBYTE,			// unsigned byte (max 255)
		INDEX_SHORT,			// 2-byte short (max 32767)
		INDEX_USHORT,			// 2-byte unsigned short (max 65535)
		INDEX_INT,				// 4-byte integer (over 2B)
		INDEX_UINT,				// 4-byte unsigned integer (over 4B)
	};

	// primitive types for drawing
	// NOTE: counter-clockwise winding for solid faces!
	enum egpPrimitiveType
	{
		PRIM_POINT,				// single point
		PRIM_LINES,				// every pair of vertices makes a line
		PRIM_LINE_STRIP,		// draws continuous line through vertices
		PRIM_LINE_LOOP,			// ditto but automatically close loop
		PRIM_TRIANGLES,			// every three vertices makes a triangle
		PRIM_TRIANGLE_STRIP,	// every three vertices makes a triangle
		PRIM_TRIANGLE_FAN,		// every three vertices makes a triangle
	};

#ifndef __cplusplus
	typedef enum egpAttributeName	egpAttributeName;
	typedef enum egpAttributeType	egpAttributeType;
	typedef enum egpIndexType		egpIndexType;
	typedef enum egpPrimitiveType	egpPrimitiveType;
#endif	// __cplusplus

	
//-----------------------------------------------------------------------------
// data structures

	// data structure representing a description of a single attribute
	struct egpAttributeDescriptor
	{
		egpAttributeName name;
		egpAttributeType type;
		unsigned int size, elems, internalType;
		const void *data;
	};

	// OpenGL vertex buffer object (VBO) descriptor
	// contains attribute types, a handle for the VBO and vertex count
	// also contains reference counter so it can't be accidentally deleted
	struct egpVertexBufferObjectDescriptor
	{
		unsigned int glhandle;
		unsigned int vertexCount, vertexSize, refCount;
		egpAttributeType attribTypes[16];
	};

	// OpenGL index/element buffer object (IBO/EBO) descriptor
	// contains index type, a handle for the IBO and index count
	// also contains reference counter
	struct egpIndexBufferObjectDescriptor
	{
		unsigned int glhandle;
		unsigned int indexCount, indexSize, refCount, internalType;
		egpIndexType indexType;
	};

#ifndef __cplusplus
	typedef struct egpAttributeDescriptor			egpAttributeDescriptor;
	typedef struct egpVertexBufferObjectDescriptor	egpVertexBufferObjectDescriptor;
	typedef struct egpIndexBufferObjectDescriptor	egpIndexBufferObjectDescriptor;
#endif	// __cplusplus


	// OpenGL vertex array object (VAO) descriptor
	// contains pointer to VBO and IBO (if used), and a handle for the VAO
	// also contains a handle to an accompanying index/element buffer object 
	//	(IBO/EBO) if one is associated with the VBO
	struct egpVertexArrayObjectDescriptor
	{
		unsigned int glhandle;
		unsigned int internalPrim;
		egpPrimitiveType primType;
		egpVertexBufferObjectDescriptor *vbo;
		egpIndexBufferObjectDescriptor *ibo;
	};

#ifndef __cplusplus
	typedef struct egpVertexArrayObjectDescriptor egpVertexArrayObjectDescriptor;
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// functions

	// generate a vertex attribute descriptor
	// calculates number of elements and size of attribute
	// 'data' param may be null if it will be set later
	egpAttributeDescriptor egpCreateAttributeDescriptor(const egpAttributeName name, const egpAttributeType type, const void *data);

	// generate an interleaved VBO given a series of attribute descriptors
	// can also generate a VBO with an IBO associated (returned in output param)
	// both cases return valid descriptor if successful
	// return is filled with zeros if unsuccessful
	// functions fail if no active attributes are passed in
	// 'num...' params cannot be zero
	egpVertexBufferObjectDescriptor egpCreateVBOInterleaved(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices);
	egpVertexBufferObjectDescriptor egpCreateVBOInterleavedIndexed(const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out);

	// generate VAO given existing buffers or raw data as per above functions
	// 'vbo' param cannot be null
	// 'ibo' param may be null if assuming non-indexed
	// the latter two functions behave the same as the above two functions
	egpVertexArrayObjectDescriptor egpCreateVAO(const egpPrimitiveType primType, egpVertexBufferObjectDescriptor *vbo, egpIndexBufferObjectDescriptor *ibo);
	egpVertexArrayObjectDescriptor egpCreateVAOInterleaved(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, egpIndexBufferObjectDescriptor *ibo);
	egpVertexArrayObjectDescriptor egpCreateVAOInterleavedIndexed(const egpPrimitiveType primType, const egpAttributeDescriptor *attribs, const unsigned int numAttribs, const unsigned int numVertices, egpVertexBufferObjectDescriptor *vbo_out, const egpIndexType indexType, const unsigned int numIndices, const void *indexData, egpIndexBufferObjectDescriptor *ibo_out);

	// activate VAO
	// 'vao' param can be null to deactivate VAO
	// NOTE: only one VAO can ever be active at a time!
	void egpActivateVAO(const egpVertexArrayObjectDescriptor *vao);

	// draw active VAO
	// separating this from activation is an optimization because it requires 
	//	fewer state changes
	// automatically selects indexed or non-indexed draw call
	// first version draws entire primitive, second allows user to draw a 
	//	portion of the vertices (partial draw)
	void egpDrawActiveVAO();
	void egpDrawActiveVAOPartial(const unsigned int first, const unsigned int count);
	void egpDrawActiveVAOInstanced(const unsigned int primCount);
	void egpDrawActiveVAOInstancedPartial(const unsigned int primCount, const unsigned int first, const unsigned int count);

	// delete objects
	// all return 1 if success, 0 if failed
	// input will be filled with zeros after function if successful
	int egpReleaseVBO(egpVertexBufferObjectDescriptor *vbo);
	int egpReleaseIBO(egpIndexBufferObjectDescriptor *ibo);
	int egpReleaseVAO(egpVertexArrayObjectDescriptor *vao);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_VERTEXBUFFER_H