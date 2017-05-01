// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwInterpolation.h"

#include <math.h>

#define lerp egpfwLerp

// ****
// lerp and lerp inverse
float egpfwLerp(const float v0, const float v1, const float param)
{
	//...
	return v0 + (v1 - v0) * param;
}

// ****
float egpfwLerpInv(const float v0, const float v1, const float v)
{
	//...
	//return egpfwLerp(v0, v1, 1.0f - v);
	return 0;
}

// ****
// Catmull-Rom spline interpolation
float egpfwCatmullRom(const float vPrev, const float v0, const float v1, const float vNext, const float param)
{
	//...
	float t = param;
	float t2 = t * t;
	float t3 = t2 * param;

	float part1 = vPrev * (-t + 2.0f * t2 - t3);
	float part2 = v0 * (2 - 5.0f * t2 + 3.0f * t3);
	float part3 = v1 * (t + 4.0f * t2 - 3.0f * t3);
	float part4 = vNext * (-1.0f * t2 + t3);

	return 0.5f * (part1 + part2 + part3 + part4);
}


// ****
// Cubic hermite spline interpolation
float egpfwCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param)
{
	//...

	float t = param;
	float t2 = t * t;
	float t3 = t2 * t;

	float part1 = v0 * (1 - 3.0 * t2 + 2.0 * t3);
	float part2 = dv0 * (t - 2.0 * t2 + t3);
	float part3 = v1 * (3.0 * t2 - 2.0 * t3);
	float part4 = dv1 * (-1.0 * t2 + t3);

	return part1 + part2 + part3 + part4;

	return 0.0f;
}


// Bezier spline interpolation
float egpfwBezier0(const float v0, const float param)
{
	// this function is complete!
	return v0;
}

// ****
float egpfwBezier1(const float v0, const float v1, const float param)
{
	//...
	return egpfwLerp(v0, v1, param);
}

// ****
float egpfwBezier2(const float v0, const float v1, const float v2, const float t)
{
	//...
	return lerp(lerp(v0, v1, t), lerp(v1, v2, t), t);
}

// ****
float egpfwBezier3(const float v0, const float v1, const float v2, const float v3, const float t)
{
	//...
	float p0p1 = lerp(v0, v1, t);
	float p1p2 = lerp(v1, v2, t);
	float p2p3 = lerp(v2, v3, t);

	return lerp(lerp(p0p1, p1p2, t), lerp(p1p2, p2p3, t), t);
}

// ****
float egpfwBezier4(const float v0, const float v1, const float v2, const float v3, const float v4, const float t)
{
	//...
	float p0p1 = lerp(v0, v1, t);
	float p1p2 = lerp(v1, v2, t);
	float p2p3 = lerp(v2, v3, t);
	float p3p4 = lerp(v3, v4, t);

	float p0p1p2 = lerp(p0p1, p1p2, t);
	float p1p2p3 = lerp(p1p2, p2p3, t);
	float p2p3p4 = lerp(p2p3, p3p4, t);

	return lerp(lerp(p0p1p2, p1p2p3, t), lerp(p1p2p3, p2p3p4, t), t);
}

// ****
float egpfwBezier(const float *v, unsigned int order, const float param)
{
	//...
	return 0.0f;
}


// ****
// vector interpolation
void egpfwLerpVector(const float *v0, const float *v1, const float param, const unsigned int numElements, float *v_out)
{
	//...
}

// ****
void egpfwCatmullRomVector(const float *vPrev, const float *v0, const float *v1, const float *vNext, const float param, const unsigned int numElements, float *v_out)
{
	//...
}

// ****
void egpfwCubicHermiteVector(const float *v0, const float *dv0, const float *v1, const float *dv1, const float param, const unsigned int numElements, float *v_out)
{
	//...
}

// ****
void egpfwBezierVector(const float *v, unsigned int order, const float param, const unsigned int numElements, float *v_out)
{
	//...
}


// ****
// table sampling
unsigned int egpfwSearchSampleTable(const float *sampleTable, const float *paramTable, const float searchParam, unsigned int numElements, float *param_out)
{
	//...
	return 0;
}


// ****
// calculate arc length
float egpfwComputeArcLengthCatmullRom(const float *vPrev, const float *v0, const float *v1, const float *vNext, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out)
{
	//...
	return 0.0f;
}

// ****
float egpfwComputeArcLengthCubicHermite(const float *v0, const float *dv0, const float *v1, const float *dv1, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out)
{
	//...
	return 0.0f;
}

// ****
float egpfwComputeArcLengthBezier(const float *v, unsigned int order, unsigned int numElements, unsigned int numSamples, int autoNormalize, float *prevSamplePtr, float *currentSamplePtr, float *sampleTable_out, float *paramTable_out)
{
	//...
	return 0.0f;
}
