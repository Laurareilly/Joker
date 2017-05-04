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
		LINES = 0,
		BEZIER,
		CATMULL_ROM,
		CUBIC_HERMITE,
		NUM_CURVES
	};

	enum KeyframeChannel
	{
		CHANNEL_POS_X = 0,
		CHANNEL_POS_Y,
		CHANNEL_POS_Z,
		CHANNEL_ROT_X,
		CHANNEL_ROT_Y,
		CHANNEL_ROT_Z,
		NUM_CHANNELS,
	};

private:
	egpVertexArrayObjectDescriptor* mVAOList;
	egpFrameBufferObjectDescriptor* mFBOList;
	egpProgram* mProgramList;

	std::array<std::vector<cbmath::vec4>, NUM_CHANNELS> mWaypointChannels;
	std::array<std::vector<cbmath::vec4>, NUM_CHANNELS> mHandles;
	//std::vector<cbmath::vec2> mHandles;
	
	cbmath::vec2 mWindowSize;
	cbmath::mat4 mLittleBoxWindowMatrix;
	cbmath::mat4 mOnScreenMatrix, mOnScreenMatrixInv;
	float mCurrentTime;
	bool mIsPaused;
	CurveType mCurrentCurve;
	KeyframeChannel mCurrentChannel;
	float mCurrentTVal;
	cbmath::vec4 mMousePos;
	
	float mWindowScale;
	float mWindowWidth;

	void resetKeyframes();

	float calculateLerp(const float v0, const float v1, float t);
	float calculateBezier();
	float calculateCatmullRom(const float vPrev, const float v0, const float v1, const float vNext, const float param);
	float calculateCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param);

public:
	SpeedControlWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs);
	~SpeedControlWindow() {};

	bool updateInput(egpMouse* m, egpKeyboard* key);
	void update(float deltaT);
	void updateWindowSize(float viewport_tw, float viewport_th, float tmpNF, float win_w, float win_h);

	float getTVal(int channel);
	CurveType getCurve() { return mCurrentCurve; }

	cbmath::mat4& getOnScreenMatrix() { return mOnScreenMatrix; }

	void renderToFBO(int* curveUniformSet, int* solidColorUniformSet);
	void renderToBackbuffer(int* textureUniformSet);
};

