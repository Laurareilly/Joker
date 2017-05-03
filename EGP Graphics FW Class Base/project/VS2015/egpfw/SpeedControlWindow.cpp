#include "SpeedControlWindow.h"
#include <egpfw/egpfw/utils/egpfwInputUtils.h>
#include <iostream>
#include <cbmath/cbtkMatrix.h>
#include <GL/glew.h>
#include "transformMatrix.h"
#include <GL/freeglut.h>

// Constant array of colors used to draw the lines on the keyframe window
const std::array<cbmath::vec4, SpeedControlWindow::NUM_CHANNELS + 1> COLORS =
{
	cbmath::vec4(1.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 1.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 0.0f, 1.0f, 1.0f),
	cbmath::vec4(1.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 1.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 0.0f, 1.0f, 1.0f),
	cbmath::vec4(1.0f, 1.0f, 1.0f, 1.0f),
};

SpeedControlWindow::SpeedControlWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs)
{
	mCurrentTime = 0.0f;
	mVAOList = vao;
	mFBOList = fbo;
	mProgramList = programs;
	mCurrentCurve = LINES;
	mIsPaused = false;
}

void SpeedControlWindow::resetKeyframes()
{
	//Clear everything, then set a point with an effective y value of 0 at the start and end.

	cbmath::vec4 zeroVec = cbmath::vec4(0.0f, mWindowSize.y / 2.0f, 0.0f, 1.0f);
	cbmath::vec4 oneVec = cbmath::vec4(mWindowSize.x, mWindowSize.y, 0.0f, 1.0f);

	for (auto& list : mWaypointChannels)
	{
		list.clear();

		list.push_back(zeroVec);
		list.push_back(oneVec);
	}
}

float SpeedControlWindow::calculateLerp(const float v0, const float v1, float t)
{
	return v0 + (v1 - v0) * t;
}

float SpeedControlWindow::calculateBezier()
{
	//int size = mWaypointChannels[mCurrentChannel].size();
	//if (size == 1)
	//{
	//	return mWaypointChannels[mCurrentChannel][0].y; //if there's only one point we're doing first order bezier so return it's only point
	//}

	//float p1 = calculateBezier();
	//float p2 = calculateBezier();

	//float p = (1 - mCurrentTime) * p1 + p2;

	//return p;

	return 1;
}

float SpeedControlWindow::calculateCatmullRom(const float vPrev, const float v0, const float v1, const float vNext, const float param)
{
	float t = param;
	float t2 = t * t;
	float t3 = t * t * t;

	float part1 = (-t + 2.0f * t2 - t3);
	float part2 = (2 - 5.0f * t2 + 3.0f * t3);
	float part3 = (t + 4.0f * t2 - 3.0f * t3);
	float part4 = -t2 + t3;

	return 0.5f * (vPrev * part1 + v0 * part2 + v1 * part3 + vNext * part4);
}

float SpeedControlWindow::calculateCubicHermite(const float v0, const float dv0, const float v1, const float dv1, const float param)
{
	float t = param;
	float t2 = t * t;
	float t3 = t2 * t;

	float part1 = v0 * (1 - 3.0 * t2 + 2.0 * t3);
	float part2 = dv0 * (t - 2.0 * t2 + t3);
	float part3 = v1 * (3.0 * t2 - 2.0 * t3);
	float part4 = dv1 * (-1.0 * t2 + t3);

	return part1 + part2 + part3 + part4;
}

bool SpeedControlWindow::updateInput(egpMouse* m, egpKeyboard* key)
{
	if (egpKeyboardIsKeyPressed(key, 'w'))
		resetKeyframes();

	if (egpKeyboardIsKeyPressed(key, ' '))
		mIsPaused = !mIsPaused;

	//Change our "mode"
	for (unsigned char c = '7', i = 0; i < ':'; ++c, ++i) //ascii : comes after 9
	{
		if (egpKeyboardIsKeyPressed(key, c))
		{
			mCurrentCurve = static_cast<CurveType>(i);
			std::cout << mCurrentCurve << std::endl;
		}
	}

	if (egpKeyboardIsKeyPressed(key, '0'))
	{
		mCurrentCurve = static_cast<CurveType>(CUBIC_HERMITE);
		std::cout << mCurrentCurve << std::endl;
	}

	for (unsigned char c = '1', i = 0; i < NUM_CHANNELS; ++c, ++i)
	{
		if (egpKeyboardIsKeyPressed(key, c))
		{
			mCurrentChannel = static_cast<KeyframeChannel>(i);
		}
	}

	cbmath::vec4 mousePos(egpMouseX(m), mWindowSize.y - egpMouseY(m), 0.0f, 1.0f);
	mousePos.x -= mWindowSize.x - mWindowWidth;
	mousePos = mOnScreenMatrixInv * mousePos;

	//Return false if we're off the quad; it means we should move the camera instead of drawing keyframes
	if (mousePos.x / mWindowSize.x > 1.0f || mousePos.y / mWindowSize.y > 1.0f)
	{
		return false;
	}

	//If the mouse is pressed down and we're in "NUM_CHANNELS" mode, scrub the keyhead
	if (egpMouseIsButtonDown(m, 0) && mCurrentChannel == NUM_CHANNELS)
	{
		mCurrentTime = (mousePos.x / mWindowSize.x) * 2.0f;
		return true;
	}

	//If the mouse is pressed and we're in a channel mode, set the t value
	if (egpMouseIsButtonPressed(m, 0) && mCurrentChannel != NUM_CHANNELS)
	{
		size_t insertIndex;
		for (insertIndex = 0; insertIndex < mWaypointChannels[mCurrentChannel].size(); insertIndex++)
		{
			if (mWaypointChannels[mCurrentChannel][insertIndex].x > mousePos.x)
				break;
		}

		//mCurrentTVal = mousePos.y / mWindowSize.y;
		std::cout << "\nT-val: " << getTVal(mCurrentChannel) << std::endl;
		mWaypointChannels[mCurrentChannel].insert(mWaypointChannels[mCurrentChannel].begin() + insertIndex, mousePos);
	}

	return true;
}

void SpeedControlWindow::update(float deltaT)
{
	if (mIsPaused)
		return;

	mCurrentTime += deltaT;

	//Wrap around because we want our whole time frame to be 2 seconds.
	if (mCurrentTime > 2.0f)
		mCurrentTime -= 2.0f;
}

void SpeedControlWindow::updateWindowSize(float viewport_tw, float viewport_th, float tmpNF, float win_w, float win_h)
{
	//Copy-pasted from the in-class example code.
	mLittleBoxWindowMatrix = cbmath::m4Identity;
	mLittleBoxWindowMatrix.m00 = 2.0f / viewport_tw;
	mLittleBoxWindowMatrix.m11 = 2.0f / viewport_th;
	mLittleBoxWindowMatrix.m22 = -1.0f / tmpNF;
	mLittleBoxWindowMatrix.m30 = -win_w / viewport_tw;
	mLittleBoxWindowMatrix.m31 = -win_h / viewport_th;

	mWindowSize.set(win_w, win_h);
	mWindowScale = 0.4f;
	mOnScreenMatrix = cbmath::makeTranslation4(0.595f, -0.59f, 0.0f) * cbmath::makeScale4(mWindowScale);
	mWindowWidth = mWindowSize.x * mWindowScale;

	//Use a handy glu function to invert the transformation matrix so that we can get mouse clicks
	if (!gluInvertMatrix(mOnScreenMatrix.m, mOnScreenMatrixInv.m))
		throw std::invalid_argument("I have no idea how this is possible, but our on-screen transformation matrix could not be inverted!");

	resetKeyframes();
}

float SpeedControlWindow::getTVal(int channel)
{
	using namespace cbmath;

	auto& list = mWaypointChannels[channel];
	

	if (list.size() == 0)
		return 0.0f;
	else if (list.size() == 1)
		return list[0].y;

	//If we have at least 2 positions, loop through the list and find 
	//the ones that are to the left and to the right of current time.
	float leftXTime, rightXTime;
	vec4 posToLeft, posToRight, prevPos, nextPos;

	for (size_t i = 0; i < list.size() - 1; i++)
	{
		leftXTime = (list[i].x / mWindowSize.x) * 2.0f;
		rightXTime = (list[i + 1].x / mWindowSize.x) * 2.0f;

		if (leftXTime <= mCurrentTime && rightXTime >= mCurrentTime)
		{
			posToLeft = list[i];
			posToRight = list[i + 1];

			int prevPosIndex = i - 1;
			if (prevPosIndex <= 0)
			{
				prevPos = list[list.size() - 1];
			}
			else
			{
				prevPos = list[i - 1];
			}

			int nextPosIndex = i + 2;
			if (nextPosIndex >= list.size())
			{
				nextPos = list[0];
			}
			else
			{
				nextPos = list[i + 2];
			}

			break;
		}
	}

	switch (channel)
	{
		case SpeedControlWindow::LINES:
			return calculateLerp(posToLeft.y, posToRight.y, mCurrentTime) / mWindowSize.y; 
		case SpeedControlWindow::BEZIER:
			return calculateBezier() / mWindowSize.y; 
		case SpeedControlWindow::CATMULL_ROM:
			return calculateCatmullRom(prevPos.y, posToLeft.y, posToRight.y, nextPos.y, mCurrentTime) / mWindowSize.y;
		case SpeedControlWindow::CUBIC_HERMITE:
			//return calculateCubicHermite(posToLeft.y, something, posToRight.y, something, mCurrentTime) / mWindowSize.y;
			return 0;
		case SpeedControlWindow::NUM_CURVES:
			return 0;
		default:
			break;
	}
}

void SpeedControlWindow::renderToFBO(int* curveUniformSet, int* solidColorUniformSet)
{
	int j;
	cbmath::vec4 *waypointPtr;
	cbmath::mat4 waypointMVP;

	egpfwActivateFBO(mFBOList + speedControlFBO);

	// clear
	glClear(GL_COLOR_BUFFER_BIT);

	int zeroTest = 0;
	int trueTest = 1;
	int twoTest = 2;
	int vecSize;

	for (size_t i = 0; i < mWaypointChannels.size(); ++i)
	{
		// draw curve
		egpActivateProgram(mProgramList + drawCurveProgram);
		egpSendUniformFloatMatrix(curveUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mLittleBoxWindowMatrix.m);

		vecSize = mWaypointChannels[i].size();

		// ship waypoint data to program, where it will be received by GS
		egpSendUniformFloat(curveUniformSet[unif_waypoint], UNIF_VEC4, vecSize, mWaypointChannels[i].data()->v);
		egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[i].v);
		egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &vecSize);
		egpSendUniformInt(curveUniformSet[unif_curveMode], (egpUniformIntType)mCurrentCurve, 1, &zeroTest); //is this how i set the curve in teh shader?
		egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

		egpActivateVAO(mVAOList + pointModel);
		egpDrawActiveVAO();

		// draw waypoints using solid color program and sphere model
		cbmath::mat4 waypointModelMatrix = cbmath::makeScale4(4.0f);
		egpActivateProgram(mProgramList + testSolidColorProgramIndex);
		egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[i].v);

		egpActivateVAO(mVAOList + sphere8x6Model);

		// draw waypoints
		for (j = 0, waypointPtr = mWaypointChannels[i].data(); j < mWaypointChannels[i].size(); ++j, ++waypointPtr)
		{
			// set position, update MVP for this waypoint and draw
			waypointModelMatrix.c3 = *waypointPtr;
			waypointMVP = mLittleBoxWindowMatrix * waypointModelMatrix;
			egpSendUniformFloatMatrix(solidColorUniformSet[unif_mvp], UNIF_MAT4, 1, 0, waypointMVP.m);
			egpDrawActiveVAO();
		}
	}

	//Draw the moving vertical line
	cbmath::vec4 points[2] = { cbmath::vec4(mCurrentTime / 2.0f * mWindowSize.x, 0.0f, 0.0f, 1.0f), cbmath::vec4(mCurrentTime / 2.0f * mWindowSize.x, mWindowSize.y, 0.0f, 1.0f) };
	egpActivateProgram(mProgramList + drawCurveProgram);
	egpSendUniformFloatMatrix(curveUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mLittleBoxWindowMatrix.m);

	egpSendUniformFloat(curveUniformSet[unif_waypoint], UNIF_VEC4, vecSize, points[0].v);
	egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[NUM_CHANNELS].v);
	egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &twoTest);
	egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
	egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

	egpActivateVAO(mVAOList + pointModel);
	egpDrawActiveVAO();
}

void SpeedControlWindow::renderToBackbuffer(int* textureUniformSet)
{
	//Draw our FBO to the backbuffer
	egpActivateProgram(mProgramList + testTextureProgramIndex);
	egpActivateVAO(mVAOList + fsqModel);
	egpfwBindColorTargetTexture(mFBOList + speedControlFBO, 0, 0);
	egpSendUniformFloatMatrix(textureUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mOnScreenMatrix.m);
	egpDrawActiveVAO();
}
