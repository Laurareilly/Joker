#pragma once
#include "egpfw/egpfw.h"

#include <vector>
#include <array>
#include <cbmath/cbtkVector.h>
#include <cbmath/cbtkMatrix.h>

struct egpMouse;
struct egpKeyboard;

class SpeedControlWindow
{
public:

	enum CurveType
	{
		BEZIER = 0,
		CATMULL_ROM,
		CUBIC_HERMITE,
		NUM_CURVES
	};

private:
	egpVertexArrayObjectDescriptor* mVAOList;
	egpFrameBufferObjectDescriptor* mFBOList;
	egpProgram* mProgramList;

	std::array<std::vector<cbmath::vec4>, NUM_CURVES> mWaypointChannels;
	
	cbmath::vec2 mWindowSize;
	cbmath::mat4 mLittleBoxWindowMatrix;
	cbmath::mat4 mOnScreenMatrix, mOnScreenMatrixInv;
	float mCurrentTime;
	bool mIsPaused;
	CurveType mCurrentCurve;
	float mCurrentTVal;
	
	float mWindowScale;
	float mWindowWidth;

	void resetKeyframes();

	float calculateBezier(std::array<std::vector<cbmath::vec4>, NUM_CURVES> points, unsigned int order, const float param);
	float calculateCatmullRom(const float vPrev, const float v0, const float v1, const float vNext, const float param);
	float calculateCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param);

public:
	SpeedControlWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs);
	~SpeedControlWindow() {};

	bool updateInput(egpMouse* m, egpKeyboard* key);
	void update(float deltaT);
	void updateWindowSize(float viewport_tw, float viewport_th, float tmpNF, float win_w, float win_h);

	float getTVal(int curve);
	CurveType getCurve() { return mCurrentCurve; }

	cbmath::mat4& getOnScreenMatrix() { return mOnScreenMatrix; }

	void renderToFBO(int* curveUniformSet, int* solidColorUniformSet);
	void renderToBackbuffer(int* textureUniformSet);
};

