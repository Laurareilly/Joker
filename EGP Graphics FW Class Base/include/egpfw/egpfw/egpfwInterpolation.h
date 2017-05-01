/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Interpolation utilities by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_INTERPOLATION_H
#define __EGPFW_INTERPOLATION_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// basic interpolation algorithms for both 1D and vectors

	// lerp and lerp inverse
	float egpfwLerp(const float v0, const float v1, const float param);
	float egpfwLerpInv(const float v0, const float v1, const float v);

	// Catmull-Rom spline interpolation
	float egpfwCatmullRom(const float vPrev, const float v0, const float v1, const float vNext, const float param);

	// Cubic hermite spline interpolation
	float egpfwCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param);
	float egpfwCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param);

	// Bezier spline interpolation
	float egpfwBezier0(const float v0, const float param);
	float egpfwBezier1(const float v0, const float v1, const float param);
	float egpfwBezier2(const float v0, const float v1, const float v2, const float param);
	float egpfwBezier3(const float v0, const float v1, const float v2, const float v3, const float param);
	float egpfwBezier4(const float v0, const float v1, const float v2, const float v3, const float v4, const float param);
	float egpfwBezier(const float *v, unsigned int order, const float param);

	// vector interpolation
	// 'numElements' is the size of the vector
	// 'v_out' is the output
	// none of the pointers can be null
	void egpfwLerpVector(const float *v0, const float *v1, const float param, const unsigned int numElements, float *v_out);
	void egpfwCatmullRomVector(const float *vPrev, const float *v0, const float *v1, const float *vNext, const float param, const unsigned int numElements, float *v_out);
	void egpfwCubicHermiteVector(const float *v0, const float *dv0, const float *v1, const float *dv1, const float param, const unsigned int numElements, float *v_out);
	void egpfwBezierVector(const float *v, unsigned int order, const float param, const unsigned int numElements, float *v_out);

	// table sampling: determine where to sample from along a spline
	// the table of values is an array of floats, you can specify how many to 
	//	skip for each search (e.g. if searching through 3D vectors, skip 3)
	// returns index in table of values, with parameter as an output pointer
	// 'sampleTable' is a table of raw sample values
	// 'paramTable' is a table of the parameters used for sampling
	// 'searchParam' is a value between 0 and 1, the param to be found
	// 'numElements' is the number of values to skip each search iteration
	// returns the index of the value to start interpolating from
	// returns the correct interpolation parameter as a pointer, 'param_out'
	unsigned int egpfwSearchSampleTable(const float *sampleTable, const float *paramTable, const float searchParam, unsigned int numElements, float *param_out);

	// utilities to calculate the arc length along a curved path segment
	// returns the total arc length of the segment
	// outputs the samples and sample parameters into the specified tables
	// user can also specify the option to auto-normalize parameters by 
	//	dividing by the total arc length
	// all of the pointers must be valid
	// 'prevSamplePtr' and 'currentSamplePtr' are external placeholders for 
	//	the sampling process (since vector length could be anything)
	// 'sampleTable_out' and 'paramTable_out' store the results
	float egpfwComputeArcLengthCatmullRom(const float *vPrev, const float *v0, const float *v1, const float *vNext, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out);
	float egpfwComputeArcLengthCubicHermite(const float *v0, const float *dv0, const float *v1, const float *dv1, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out);
	float egpfwComputeArcLengthBezier(const float *v, unsigned int order, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_INTERPOLATION_H