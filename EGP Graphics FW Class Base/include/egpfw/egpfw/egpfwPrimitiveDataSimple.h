/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Utilities for drawing simple primitives in immediate mode or getting 
	their data to be used for retained mode drawing.
*/

#ifndef __EGPFW_PRIMITIVEDATASIMPLE_H
#define __EGPFW_PRIMITIVEDATASIMPLE_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------

	
	// draw coordinate axes using immediate mode
	void egpfwDrawAxesImmediate(const float *mvp, const int mvpLoc);

	// draw colored triangle using immediate mode
	// position and color attributes each have 3 elements
	void egpfwDrawColoredTriangleImmediate(const float *mvp, const int mvpLoc);

	// draw colored unit quad using immediate mode
	// should draw quad as a TRIANGLE STRIP (quad is deprecated)
	void egpfwDrawColoredUnitQuadImmediate(const float *mvp, const int mvpLoc);

	// draw unit quad with texture coordinates using immediate mode
	// texture coordinates have 2 elements
	// should draw quad as a TRIANGLE STRIP (quad is deprecated)
	void egpfwDrawTexturedUnitQuadImmediate(const float *mvp, const int mvpLoc);


//-----------------------------------------------------------------------------


	// get data for quad that can be colored or textured
	// data should be arranged as TRIANGLE STRIP
	// positions and colors have 3 elements each, texture coordinates have 2
	const float *egpfwGetUnitQuadPositions();
	const float *egpfwGetUnitQuadColors();
	const float *egpfwGetUnitQuadTexcoords();
	unsigned int egpfwGetUnitQuadVertexCount();


	const float *getUnitDiscPosition();
	const float *getUnitDiscColors();
	const float *getUnitDiscTexcoords();
	unsigned int getUnitDiscVertexCount();
	unsigned int getUnitDiscIndexCount();
	const unsigned int *egpGetUnitDiscIndices();

//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_PRIMITIVEDATASIMPLE_H