// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwPrimitiveDataSimple.h"


// OpenGL
#ifdef _WIN32
#include "GL/glew.h"
#else	// !_WIN32
#include <OpenGL/gl.h>
#endif	// _WIN32


//------------------------------------------------------------------------------


// draw coordinate axes using immediate mode
// **WARNING: DO NOT EVER USE IMMEDIATE MODE FOR FULL-SCALE MODELS
void egpfwDrawAxesImmediate(const float *mvp, const int mvpLoc)
{
	// for immediate rendering, load mvp matrix onto stack
	glPushMatrix();
	glLoadMatrixf(mvp);

	// if shaders are being used, mvp should be sent as a uniform
	// this is still considered immediate mode because the data 
	//	is discarded immediately after it is used
	glUniformMatrix4fv(mvpLoc, 1, 0, mvp);

	// start drawing primitives
	glBegin(GL_LINES);

	// send attributes 1-by-1
	// glVertex is used to send position and finish current vertex
	// (the last attribute values sent will be used for this vertex)
	glColor3f (1.0f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);

	glColor3f (0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);

	glColor3f (0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);

	// stop drawing primitives
	glEnd();

	// remove mvp matrix from stack
	glPopMatrix();
}


// ****
// draw colored triangle using immediate mode
// position and color attributes each have 3 elements
// **WARNING: DO NOT EVER USE IMMEDIATE MODE FOR FULL-SCALE MODELS
void egpfwDrawColoredTriangleImmediate(const float *mvp, const int mvpLoc)
{
	glPushMatrix();
	//...
	glPopMatrix();
}


// ****
// draw colored unit quad using immediate mode
// should draw quad as a TRIANGLE STRIP (quad is deprecated)
// **WARNING: DO NOT EVER USE IMMEDIATE MODE FOR FULL-SCALE MODELS
void egpfwDrawColoredUnitQuadImmediate(const float *mvp, const int mvpLoc)
{
	glPushMatrix();
	//...
	glPopMatrix();
}


// ****
// draw unit quad with texture coordinates using immediate mode
// texture coordinates have 2 elements
// should draw quad as a TRIANGLE STRIP (quad is deprecated)
// **WARNING: DO NOT EVER USE IMMEDIATE MODE FOR FULL-SCALE MODELS
void egpfwDrawTexturedUnitQuadImmediate(const float *mvp, const int mvpLoc)
{
	glPushMatrix();
	//...
	glPopMatrix();
}


//-----------------------------------------------------------------------------

/*
// ****
// raw quad data
// data should be arranged as TRIANGLE STRIP: use at most 4 vertices!
#define quadNumVertices 4
const float fwUnitQuadPositions[quadNumVertices * 3] = {
  -1.0f, -1.0f, 0.0f,
  +1.0f, -1.0f, 0.0f,
  -1.0f, +1.0f, 0.0f,
  +1.0f, +1.0f, 0.0f,
};
const float fwUnitQuadColors[quadNumVertices * 3] = {
  0.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  1.0f, 1.0f, 0.0f,
};
const float fwUnitQuadTexcoords[quadNumVertices * 2] = {
  0.0f, 0.0f,
  1.0f, 0.0f,
  0.0f, 1.0f,
  1.0f, 1.0f,
};
*/

//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------

// ****
// raw quad data
// data should be arranged as TRIANGLE STRIP: use at most 4 vertices!
#define quadNumVertices 4
const float fwUnitQuadPositions[quadNumVertices * 3] = {
	-1.0f,-1.0f,0.0f,
	1.0f,-1.0f,0.0f,
	-1.0f,1.0f,0.0f,
	1.0f,1.0f,0.0f
};
const float fwUnitQuadColors[quadNumVertices * 3] = {
	0.0f,0.0f,0.0f,
	1.0f,0.0f,0.0f,
	0.0f,0.0f,0.0f,
	1.0f,0.0f,0.0f,
};
const float fwUnitQuadTexcoords[quadNumVertices * 2] = {
	0.0f, 0.0f,
	1.0f, 0.0f,
	0.0f, 1.0f,
	1.0f, 1.0f
};


//-----------------------------------------------------------------------------

// get data for quad that can be colored or textured
const float *egpfwGetUnitQuadPositions()
{
	// this function is complete!
	return fwUnitQuadPositions;
}

const float *egpfwGetUnitQuadColors()
{
	// this function is complete!
	return fwUnitQuadColors;
}

const float *egpfwGetUnitQuadTexcoords()
{
	// this function is complete!
	return fwUnitQuadTexcoords;
}

unsigned int egpfwGetUnitQuadVertexCount()
{
	// this function is complete!
	return quadNumVertices;
}


#define discNumVertices 9
const float fwUnitDiscPositions[discNumVertices * 3] = {
	0.0f, 0.0f, 0.0f,	//center			0
	0.5f, -0.5f, 0.0f,	//down, right		1
	0.75f, 0.0f, 0.0f,	//right				2
	0.5f, 0.5f, 0.0f,	//up, right			3
	0.0f, 0.75f, 0.0f,	//up				4
	-0.5f, 0.5f, 0.0f,	//up, left			5
	-0.75f, 0.0f, 0.0f,	//left				6
	-0.5f, -0.5f, 0.0f,	//down, left		7
	0.0f, -0.75, 0.0f,	//down				8
};

#define discNumIndicies 24
const unsigned int fwUnitDiscIndicies[discNumIndicies] = {
	0, 1, 2,
	0, 2, 3,
	0, 3, 4,
	0, 4, 5,
	0, 5, 6,
	0, 6, 7,
	0, 7, 8,
	0, 8, 1
};

const float fwUnitDiscColors[discNumVertices * 3] = {
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
	1.0f, 1.0f, 1.0f,
};
const float fwUnitDiscTexcoords[discNumVertices * 2] = {
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
	0.0f, 0.0f,
};

const float* getUnitDiscPosition()
{
	return fwUnitDiscPositions;
}

const float* getUnitDiscColors()
{
	return fwUnitDiscColors;
}

const float* getUnitDiscTexcoords()
{
	return fwUnitDiscTexcoords;
}

unsigned getUnitDiscVertexCount()
{
	return discNumVertices;
}

unsigned getUnitDiscIndexCount()
{
	return discNumIndicies;
}

const unsigned* egpGetUnitDiscIndices()
{
	return fwUnitDiscIndicies;
}

//-----------------------------------------------------------------------------
