#include "SpeedControlWindow.h"
#include <egpfw/egpfw/utils/egpfwInputUtils.h>
#include <iostream>
#include <cbmath/cbtkMatrix.h>
#include <GL/glew.h>
#include "transformMatrix.h"
#include <GL/freeglut.h>

// Constant array of colors used to draw the lines on the keyframe window
const std::array<cbmath::vec4, SpeedControlWindow::NUM_CURVES + 1> COLORS =
{
	cbmath::vec4(1.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 1.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 0.0f, 1.0f, 1.0f),
	cbmath::vec4(1.0f, 0.9f, 0.0f, 1.0f),
};

SpeedControlWindow::SpeedControlWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs)
{
	mCurrentTime = 0.0f;
	mVAOList = vao;
	mFBOList = fbo;
	mProgramList = programs;
	mCurrentCurve = BEZIER;
	mIsPaused = false;
}

void SpeedControlWindow::resetKeyframes()
{
	//Clear everything, then set a point with an effective y value of 0 at the start and end.

	cbmath::vec4 zeroVec = cbmath::vec4(0.0f, mWindowSize.y / 2.0f, 0.0f, 1.0f);
	cbmath::vec4 oneVec = cbmath::vec4(mWindowSize.x, mWindowSize.y / 2.0f, 0.0f, 1.0f);

	for (auto& list : mWaypointChannels)
	{
		list.clear();

		list.push_back(zeroVec);
		list.push_back(oneVec);
	}
}

bool SpeedControlWindow::updateInput(egpMouse* m, egpKeyboard* key)
{
	if (egpKeyboardIsKeyPressed(key, 'y'))
		resetKeyframes();

	if (egpKeyboardIsKeyPressed(key, ' '))
		mIsPaused = !mIsPaused;

	//Change our "mode"
	for (unsigned char c = '1', i = 0; i <= NUM_CURVES; ++c, ++i)
	{
		if (egpKeyboardIsKeyPressed(key, c))
			mCurrentCurve = static_cast<CurveType>(i);
	}

	cbmath::vec4 mousePos(egpMouseX(m), mWindowSize.y - egpMouseY(m), 0.0f, 1.0f);
	mousePos.x -= mWindowSize.x - mWindowWidth;
	mousePos = mOnScreenMatrixInv * mousePos;

	//Return false if we're off the quad; it means we should move the camera instead of drawing keyframes
	if (mousePos.x / mWindowSize.x > 1.0f || mousePos.y / mWindowSize.y > 1.0f)
	{
		return false;
	}

	//If the mouse is pressed down and we're in "NUM_CURVES" mode, scrub the keyhead
	if (egpMouseIsButtonDown(m, 0) && mCurrentCurve == NUM_CURVES)
	{
		mCurrentTime = (mousePos.x / mWindowSize.x) * 2.0f;
		return true;
	}

	//If the mouse is pressed and we're in a channel mode, set the t value
	if (egpMouseIsButtonPressed(m, 0) && mCurrentCurve != NUM_CURVES)
	{
		size_t insertIndex;
		for (insertIndex = 0; insertIndex < mWaypointChannels[mCurrentCurve].size(); insertIndex++)
		{
			if (mWaypointChannels[mCurrentCurve][insertIndex].x > mousePos.x)
				break;
		}

		mCurrentTVal = mousePos.y / mWindowSize.y;
		std::cout << "\nT-val: " << mCurrentTVal << std::endl;
		mWaypointChannels[mCurrentCurve].insert(mWaypointChannels[mCurrentCurve].begin() + insertIndex, mousePos);
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
		egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
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
	egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[NUM_CURVES].v);
	egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &twoTest);
	egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
	egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

	egpActivateVAO(mVAOList + pointModel);
	egpDrawActiveVAO();

	//Draw the x axis
	//points[0] = cbmath::vec4(0.0f, mWindowSize.y / 2.0f, 0.0f, 1.0f);
	//points[1] = cbmath::vec4(mWindowSize.x, mWindowSize.y / 2.0f, 0.0f, 1.0f);

	//egpActivateProgram(mProgramList + drawCurveProgram);
	//egpSendUniformFloatMatrix(curveUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mLittleBoxWindowMatrix.m);

	//egpSendUniformFloat(curveUniformSet[unif_waypoint], UNIF_VEC4, vecSize, points[0].v);
	//egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[2].v);
	//egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &twoTest);
	//egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
	//egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

	//egpActivateVAO(mVAOList + pointModel);
	//egpDrawActiveVAO();

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
