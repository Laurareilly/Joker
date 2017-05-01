/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Handy data for complex primitives by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_PRIMITIVEDATA_H
#define __EGPFW_UTILS_PRIMITIVEDATA_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------

	// utilities to get raw data pertaining to tricky primitives
	// IMPORTANT: vertices and normals each have 3 floats, 
	//	texture coordinates have 2 floats

	int egpInitializeAxes();
	const float *egpGetAxesPositions();
	const float *egpGetAxesColors();
	unsigned int egpGetAxesVertexCount();

	// sphere low-res (8 slices x 6 stacks): 
	int egpInitializeSphere8x6();
	const float *egpGetSphere8x6Positions();
	const float *egpGetSphere8x6Normals();
	const float *egpGetSphere8x6Colors();
	const float *egpGetSphere8x6Texcoords();
	unsigned int egpGetSphere8x6VertexCount();

	// sphere high-res (32 slices x 24 stacks): 
	int egpInitializeSphere32x24();
	const float *egpGetSphere32x24Positions();
	const float *egpGetSphere32x24Normals();
	const float *egpGetSphere32x24Colors();
	const float *egpGetSphere32x24Texcoords();
	unsigned int egpGetSphere32x24VertexCount();

	// cube standard: 
	int egpInitializeCube();
	const float *egpGetCubePositions();
	const float *egpGetCubeNormals();
	const float *egpGetCubeColors();
	const float *egpGetCubeTexcoords();
	unsigned int egpGetCubeVertexCount();

	// cube indexed: 
	int egpInitializeCubeIndexed();
	const float *egpGetCubeIndexedPositions();
	const float *egpGetCubeIndexedNormals();
	const float *egpGetCubeIndexedColors();
	const float *egpGetCubeIndexedTexcoords();
	const unsigned int *egpGetCubeIndices();
	unsigned int egpGetCubeIndexedVertexCount();
	unsigned int egpGetCubeIndexCount();

	// wireframe cube standard: 
	int egpInitializeWireCube();
	const float *egpGetWireCubePositions();
	unsigned int egpGetWireCubeVertexCount();

	// wire cube indexed: 
	int egpInitializeWireCubeIndexed();
	const float *egpGetWireCubeIndexedPositions();
	const unsigned int *egpGetWireCubeIndices();
	unsigned int egpGetWireCubeIndexedVertexCount();
	unsigned int egpGetWireCubeIndexCount();


//-----------------------------------------------------------------------------

	// draw 3D primitives using immediate mode
	void egpDrawSphere8x6Immediate(const float *mvp, const int mvpLoc, const float r, const float g, const float b);
	void egpDrawSphere32x24Immediate(const float *mvp, const int mvpLoc, const float r, const float g, const float b);
	void egpDrawCubeImmediate(const float *mvp, const int mvpLoc, const int isIndexed, const float r, const float g, const float b);
	void egpDrawWireCubeImmediate(const float *mvp, const int mvpLoc, const int isIndexed, const float r, const float g, const float b);

	// draw a series of samples in immediate mode
	void egpDrawSamplesImmediate(const float *v, unsigned int numElems, unsigned int numSamples, const float *mvp, const int mvpLoc, const float r, const float g, const float b);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_PRIMITIVEDATA_H