// By Dan Buckstein
// Modified by: James Keats with permission from author.
#include "egpfw/egpfw/egpfwOBJLoader.h"


#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glew.h>


#define BUFFER_OFFSET_BYTE(p,n) ((char *)(p) + n)


// helper structures
union float2
{
	struct { float f0, f1; };
	float f[2];
};
union float3
{
	struct { float f0, f1, f2; };
	float f[3];
};
union float4
{
	struct { float f0, f1, f2, f3; };
	float f[4];
};
union int4
{
	struct { int i0, i1, i2, i3; };
	int i[4];
};
union face
{
	struct { unsigned int v0, v1, v2, vt0, vt1, vt2, vn0, vn1, vn2; };
	struct { unsigned int v[3], vt[3], vn[3]; };
};

struct uniqueTriFace;
struct uniqueVertex;
struct uniqueTexcoord;
struct uniqueNormal;

#ifndef __cplusplus
typedef union float2			float2;
typedef union float3			float3;
typedef union float4			float4;
typedef union int4				int4;
typedef union face				face;
typedef struct uniqueTriFace	uniqueTriFace;
typedef struct uniqueVertex		uniqueVertex;
typedef struct uniqueTexcoord	uniqueTexcoord;
typedef struct uniqueNormal		uniqueNormal;
#endif	// __cplusplus

struct interleave
{
	float3 pos, nor;
	float2 texcoord;
};

typedef struct interleave interleave;


//-----------------------------------------------------------------------------


#define BUFFER_SIZE 2048
#define LINE_SIZE 64

// ****
// load triangulated OBJ file
egpTriOBJDescriptor egpfwLoadTriangleOBJ(const char *objPath, const egpMeshNormalMode normalMode, const double globalScale)
{
	egpTriOBJDescriptor obj = { 0 };

	unsigned int numVerticies, numTexcoords, numNormals, numFaces;
	float3 vertexBuffer[BUFFER_SIZE]; //load a (relatively) large buffer for the verticies
	float2 vertexTexBuffer[BUFFER_SIZE]; //load a (relatively) large buffer for the verticies
	float3 vertexNorBuffer[BUFFER_SIZE]; //load a (relatively) large buffer for the verticies
	face faceBuffer[BUFFER_SIZE]; //load a (relatively) large buffer for the faces
	
	FILE* objFile;
	char lineBuffer[LINE_SIZE];
	float3 currentF3 = { 0 };
	float2 currentF2 = { 0 };
	face f = { 0 };

	int i = 0;

	//Load our file handle
	objFile = fopen(objPath, "r");
	if (objFile == NULL)
	{
		printf("Unable to open file %s.\n", objPath);
		return obj;
	}

	//Skip the junk, go to the first line that has a vertex.
	while (lineBuffer[0] != 'v' && fgets(lineBuffer, 64, objFile) != NULL) {}

	//Load all the verticies into vertexBuffer
	while (lineBuffer[0] == 'v' && lineBuffer[1] == ' ')
	{
		//Scan the line data into currentF3 then put that in the buffer.
		sscanf(lineBuffer, "%*s %f %f %f", &currentF3.f0, &currentF3.f1, &currentF3.f2);
		vertexBuffer[i] = currentF3;
		i++;

		if (i >= BUFFER_SIZE)
		{
			printf("Number of verticies in .obj overflowed buffer: %s", objPath);
			return obj;
		}

		if (fgets(lineBuffer, LINE_SIZE, objFile) == NULL)
			break; //hit the end of the file early
	}

	numVerticies = i;
	i = 0;

	//Load all the texcoords into vertexTexBuffer
	while (lineBuffer[0] == 'v' && lineBuffer[1] == 't')
	{
		//Scan the line data into currentF2 then put that in the buffer.
		sscanf(lineBuffer, "%*s %f %f", &currentF2.f0, &currentF2.f1);
		vertexTexBuffer[i] = currentF2;
		i++;

		if (i >= BUFFER_SIZE)
		{
			printf("Number of texcoords in .obj overflowed buffer: %s", objPath);
			return obj;
		}
		
		if (fgets(lineBuffer, LINE_SIZE, objFile) == NULL)
			break; //hit the end of the file early
	}

	numTexcoords = i;
	i = 0;
	
	//Load all the normals into vertexNorBuffer
	while (lineBuffer[0] == 'v' && lineBuffer[1] == 'n')
	{
		//Scan the line data into currentF3 then put that in the buffer.
		sscanf(lineBuffer, "%*s %f %f %f", &currentF3.f0, &currentF3.f1, &currentF3.f2);
		vertexNorBuffer[i] = currentF3;
		i++;

		if (i >= BUFFER_SIZE)
		{
			printf("Number of normals in .obj overflowed buffer: %s", objPath);
			return obj;
		}

		if (fgets(lineBuffer, LINE_SIZE, objFile) == NULL)
			break; //hit the end of the file early
	}

	numNormals = i;
	i = 0;

	//skip the junk in the middle and go to the faces
	while (lineBuffer[0] != 'f' && fgets(lineBuffer, 64, objFile) != NULL) {}

	//Load all the faces into faceBuffer
	while (lineBuffer[0] == 'f' && lineBuffer[1] == ' ')
	{
		//Scan the line data into temporary face then put that in the buffer.
		sscanf(lineBuffer,
			"%*s %i %*c %i %*c %i %i %*c %i %*c %i %i %*c %i %*c %i",
			&f.v0, &f.vt0, &f.vn0,
			&f.v1, &f.vt1, &f.vn1,
			&f.v2, &f.vt2, &f.vn2);

		//For some stupid reason, the faces are 1-indexed.
		//Decrement everything so that they're 0-indexed.
		--f.v0; --f.vt0; --f.vn0;
		--f.v1; --f.vt1; --f.vn1;
		--f.v2; --f.vt2; --f.vn2;

		faceBuffer[i] = f;
		i++;

		if (i >= BUFFER_SIZE)
		{
			printf("Number of faces in .obj overflowed buffer: %s", objPath);
			return obj;
		}

		if (fgets(lineBuffer, LINE_SIZE, objFile) == NULL)
			break;
	}

	numFaces = i;
	i = 0;

	//Alright, we finally have all of our data. What's our total size?
	obj.dataSize = 
		sizeof(float3) * numVerticies +
		sizeof(float2) * numTexcoords +
		sizeof(float3) * numNormals +
		sizeof(face) * numFaces;
	
	obj.data = (void*)malloc(obj.dataSize);

	//We'll put the faces at 0 since they don't have an enum. Where do the chunks after that go?
	obj.attribOffset[ATTRIB_POSITION] =	sizeof(face) * numFaces;
	obj.attribOffset[ATTRIB_TEXCOORD] =	obj.attribOffset[ATTRIB_POSITION] + sizeof(float3) * numVerticies;
	obj.attribOffset[ATTRIB_NORMAL] =	obj.attribOffset[ATTRIB_TEXCOORD] + sizeof(float2) * numTexcoords;

	//Everything is set up. Now, copy our data directly into the buffer.
	memcpy(obj.data, faceBuffer, sizeof(face) * numFaces);
	memcpy(BUFFER_OFFSET_BYTE(obj.data, obj.attribOffset[ATTRIB_POSITION]), vertexBuffer, sizeof(float3) * numVerticies);
	memcpy(BUFFER_OFFSET_BYTE(obj.data, obj.attribOffset[ATTRIB_TEXCOORD]), vertexTexBuffer, sizeof(float2) * numTexcoords);
	memcpy(BUFFER_OFFSET_BYTE(obj.data, obj.attribOffset[ATTRIB_NORMAL]), vertexNorBuffer, sizeof(float3) * numNormals);

	return obj;
}


// ****
// convert OBJ to VAO & VBO
int egpfwCreateVAOFromOBJ(const egpTriOBJDescriptor *obj, egpVertexArrayObjectDescriptor *vao_out, egpVertexBufferObjectDescriptor *vbo_out)
{
	//Get the start and end + 1 of the face data so that we can loop through it.
	void* faceStart = obj->data;
	void* faceEnd = BUFFER_OFFSET_BYTE(obj->data, obj->attribOffset[ATTRIB_POSITION]);

	//Find our positions, normals, and texcoords inside the data buffer.
	float3* data_positions = (float3*)(BUFFER_OFFSET_BYTE(obj->data, obj->attribOffset[ATTRIB_POSITION]));
	float3* data_normals = (float3*)(BUFFER_OFFSET_BYTE(obj->data, obj->attribOffset[ATTRIB_NORMAL]));
	float2* data_texcoords = (float2*)(BUFFER_OFFSET_BYTE(obj->data, obj->attribOffset[ATTRIB_TEXCOORD]));

	//number of faces is the distance between the start and end of the face part of the buffer.
	unsigned int numberOfFaces = (unsigned int)((char*)faceEnd - (char*)faceStart) / sizeof(face);
	unsigned int vertexCount = numberOfFaces * 3;
	

	/* This is the complicated way of doing it.
	//Create a pre-interleaved buffer
	unsigned int interleavedBufferSize = sizeof(interleave) * vertexCount;
	interleave* interleavedBuffer = (interleave*)malloc(interleavedBufferSize);

	face* fdata;
	interleave idata;
	int i = 0;

	//Walk down the face part of the data buffer and grab the relevant data from the other parts.
	for (void* walker = faceStart; walker != faceEnd; walker = BUFFER_OFFSET_BYTE(walker, sizeof(face)))
	{
		fdata = (face*)walker;
		
		//Grab the pos, normal, and texcoord for this vertex
		idata.pos = data_positions[fdata->v0];
		idata.nor = data_normals[fdata->vn0];
		idata.texcoord = data_texcoords[fdata->vt0];

		//Put it in the buffer
		interleavedBuffer[i] = idata;
		i++;

		//Repeat 2 more times per face.

		idata.pos = data_positions[fdata->v1];
		idata.nor = data_normals[fdata->vn1];
		idata.texcoord = data_texcoords[fdata->vt1];

		interleavedBuffer[i] = idata;
		i++;

		idata.pos = data_positions[fdata->v2];
		idata.nor = data_normals[fdata->vn2];
		idata.texcoord = data_texcoords[fdata->vt2];

		interleavedBuffer[i] = idata;
		i++;
	}

	//generate a VBO using GL calls
	glGenBuffers(1, &vbo_out->glhandle);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_out->glhandle);
	glBufferData(GL_ARRAY_BUFFER, interleavedBufferSize, (void*)interleavedBuffer, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Set up the other vbo variables how I assume they're supposed to be used.
	vbo_out->attribTypes[ATTRIB_NORMAL] = 1;
	vbo_out->attribTypes[ATTRIB_TEXCOORD] = 1;
	vbo_out->attribTypes[ATTRIB_POSITION] = 1;
	vbo_out->vertexSize = sizeof(float3);
	vbo_out->vertexCount = vertexCount;

	//Bind the vbo to a vao using GL calls
	glGenVertexArrays(1, &vao_out->glhandle);
	glBindVertexArray(vao_out->glhandle);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_out->glhandle);
	
	glEnableVertexAttribArray(ATTRIB_POSITION);
	glVertexAttribPointer(ATTRIB_POSITION,
		3, GL_FLOAT, GL_FALSE, sizeof(float3) + sizeof(float3) + sizeof(float2), (void*)0);
	
	glEnableVertexAttribArray(ATTRIB_NORMAL);
	glVertexAttribPointer(ATTRIB_NORMAL,
		3, GL_FLOAT, GL_FALSE, sizeof(float3) + sizeof(float3) + sizeof(float2), (void*)sizeof(float3));

	glEnableVertexAttribArray(ATTRIB_TEXCOORD);
	glVertexAttribPointer(ATTRIB_TEXCOORD,
		3, GL_FLOAT, GL_FALSE, sizeof(float3) + sizeof(float3) + sizeof(float2), (void*)(sizeof(float3) + sizeof(float3)));

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	//Set up the other vao variables how I assume they're supposed to be used.
	vao_out->vbo = vbo_out;
	vao_out->primType = PRIM_TRIANGLES;
	vao_out->ibo = NULL;
	
	//Free the temporary buffer now that everything has been sent off to the GPU.
	free(interleavedBuffer);
	/**/

	//* This is the "simple" way of doing it.
	//Create three attribute buffers.
	float3* posBuffer = malloc(sizeof(float3) * vertexCount);
	float3* norBuffer = malloc(sizeof(float3) * vertexCount);
	float2* texBuffer = malloc(sizeof(float2) * vertexCount);
	face* fdata;
	int i = 0;

	//Walk down the face part of the data buffer and grab the relevant data from the other parts.
	for (void* walker = faceStart; walker != faceEnd; walker = BUFFER_OFFSET_BYTE(walker, sizeof(face)))
	{
		//Grab the data from the walker's current position.
		fdata = (face*)walker;

		//put the face's first vertex in
		posBuffer[i] = data_positions[fdata->v0];
		norBuffer[i] = data_normals[fdata->vn0];
		texBuffer[i] = data_texcoords[fdata->vt0];
		i++;

		//put the face's second vertex in
		posBuffer[i] = data_positions[fdata->v1];
		norBuffer[i] = data_normals[fdata->vn1];
		texBuffer[i] = data_texcoords[fdata->vt1];
		i++;

		//put the face's third vertex in
		posBuffer[i] = data_positions[fdata->v2];
		norBuffer[i] = data_normals[fdata->vn2];
		texBuffer[i] = data_texcoords[fdata->vt2];
		i++;
	}

	//Create our attributes and bind them
	egpAttributeDescriptor attribs[] = 
	{
		egpCreateAttributeDescriptor(ATTRIB_POSITION, ATTRIB_VEC3, posBuffer),
		egpCreateAttributeDescriptor(ATTRIB_NORMAL, ATTRIB_VEC3, norBuffer),
		egpCreateAttributeDescriptor(ATTRIB_TEXCOORD, ATTRIB_VEC2, texBuffer),
	};

	*vbo_out = egpCreateVBOInterleaved(attribs, 3, vertexCount);
	*vao_out = egpCreateVAO(PRIM_TRIANGLES, vbo_out, NULL);

	//Free our buffers. Presumably, the data has been sent off to the GPU so we don't need it anymore.
	free(posBuffer);
	free(norBuffer);
	free(texBuffer);
	/**/

	return 0;
}


// ****
// free obj data
int egpfwReleaseOBJ(egpTriOBJDescriptor *obj)
{
	//...
	free(obj->data);
	return 0;
}


// ****
// save/load binary
int egpfwSaveBinaryOBJ(const egpTriOBJDescriptor *obj, const char *binPath)
{
	//...
	return 0;
}

egpTriOBJDescriptor egpfwLoadBinaryOBJ(const char *binPath)
{
	egpTriOBJDescriptor obj = { 0 };
	//...
	return obj;
}


// these functions are complete!
// get attribute data from OBJ
const void *egpfwGetOBJAttributeData(const egpTriOBJDescriptor *obj, const egpAttributeName attrib)
{
	if (obj && obj->data && obj->attribOffset[attrib])
		return BUFFER_OFFSET_BYTE(obj->data, obj->attribOffset[attrib]);
	return 0;
}

unsigned int egpfwGetOBJNumVertices(const egpTriOBJDescriptor *obj)
{
	if (obj && obj->data)
		return *((unsigned int *)obj->data);
	return 0;
}
