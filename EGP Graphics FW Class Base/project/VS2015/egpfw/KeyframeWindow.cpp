#include "KeyframeWindow.h"
#include <egpfw/egpfw/utils/egpfwInputUtils.h>
#include <iostream>
#include <cbmath/cbtkMatrix.h>
#include <GL/glew.h>
#include "transformMatrix.h"
#include <GL/freeglut.h>

// Constant array of colors used to draw the lines on the keyframe window
const std::array<cbmath::vec4, KeyframeWindow::NUM_OF_CHANNELS + 1> COLORS = 
{
	cbmath::vec4(1.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 1.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 0.0f, 1.0f, 1.0f),
	cbmath::vec4(1.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 1.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 0.0f, 1.0f, 1.0f),
	cbmath::vec4(1.0f, 1.0f, 1.0f, 1.0f),
};

KeyframeWindow::KeyframeWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs)
{
	mCurrentTime = 0.0f;
	mVAOList = vao;
	mFBOList = fbo;
	mProgramList = programs;
	mCurrentChannel = CHANNEL_POS_X;
	mIsPaused = false;
}

KeyframeWindow::~KeyframeWindow()
{

}

void KeyframeWindow::resetKeyframes()
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

bool KeyframeWindow::updateInput(egpMouse* m, egpKeyboard* key)
{
	if (egpKeyboardIsKeyPressed(key, 'y'))
		resetKeyframes();

	if (egpKeyboardIsKeyPressed(key, ' '))
		mIsPaused = !mIsPaused;

	//Change our "mode"
	for (unsigned char c = '1', i = 0; i <= NUM_OF_CHANNELS; ++c, ++i)
	{
		if (egpKeyboardIsKeyPressed(key, c))
			mCurrentChannel = static_cast<KeyframeChannel>(i);
	}

	cbmath::vec4 mousePos(egpMouseX(m), mWindowSize.y - egpMouseY(m), 0.0f, 1.0f);
	mousePos = mOnScreenMatrixInv * mousePos;

	//Return false if we're off the quad; it means we should move the camera instead of drawing keyframes
	if (mousePos.x / mWindowSize.x > 1.0f || mousePos.y / mWindowSize.y > 1.0f)
		return false;

	//If the mouse is pressed down and we're in "NUM_OF_CHANNELS" mode, scrub the keyhead
	if (egpMouseIsButtonDown(m, 0) && mCurrentChannel == NUM_OF_CHANNELS)
	{
		mCurrentTime = (mousePos.x / mWindowSize.x) * 2.0f;
		return true;
	}

	//If the mouse is pressed and we're in a channel mode, add a keyframe 
	if (egpMouseIsButtonPressed(m, 0) && mCurrentChannel != NUM_OF_CHANNELS)
	{
		size_t insertIndex;
		for (insertIndex = 0; insertIndex < mWaypointChannels[mCurrentChannel].size(); insertIndex++)
		{
			if (mWaypointChannels[mCurrentChannel][insertIndex].x > mousePos.x)
				break;
		}

		mWaypointChannels[mCurrentChannel].insert(mWaypointChannels[mCurrentChannel].begin() + insertIndex, mousePos);
	}

	return true;
}

void KeyframeWindow::update(float deltaT)
{
	//tick tock
	if (mIsPaused)
		return;

	mCurrentTime += deltaT;

	//Wrap around because we want our whole time frame to be 2 seconds.
	if (mCurrentTime > 2.0f)
		mCurrentTime -= 2.0f;
}

void KeyframeWindow::updateWindowSize(float viewport_tw, float viewport_th, float tmpNF, float win_w, float win_h)
{
	//Copy-pasted from the in-class example code.
	mLittleBoxWindowMatrix = cbmath::m4Identity;
	mLittleBoxWindowMatrix.m00 = 2.0f / viewport_tw;
	mLittleBoxWindowMatrix.m11 = 2.0f / viewport_th;
	mLittleBoxWindowMatrix.m22 = -1.0f / tmpNF;
	mLittleBoxWindowMatrix.m30 = -win_w / viewport_tw;
	mLittleBoxWindowMatrix.m31 = -win_h / viewport_th;

	mWindowSize.set(win_w, win_h);
	mOnScreenMatrix = cbmath::makeTranslation4(-0.595f, -0.59f, 0.0f) * cbmath::makeScale4(0.4f);

	//Use a handy glu function to invert the transformation matrix so that we can get mouse clicks
	if (!gluInvertMatrix(mOnScreenMatrix.m, mOnScreenMatrixInv.m))
		throw std::invalid_argument("I have no idea how this is possible, but our on-screen transformation matrix could not be inverted!");

	resetKeyframes();
}

float KeyframeWindow::getValAtCurrentTime(KeyframeChannel c)
{
	using namespace cbmath;

	auto& list = mWaypointChannels[c];

	if (list.size() == 0)
		return 0.0f;
	else if (list.size() == 1)
		return list[0].y;

	//If we have at least 2 positions, loop through the list and find 
	//the ones that are to the left and to the right of current time.
	float leftXTime, rightXTime;
	vec4 posToLeft, posToRight;

	for (size_t i = 0; i < list.size() - 1; i++)
	{
		leftXTime = (list[i].x / mWindowSize.x) * 2.0f;
		rightXTime = (list[i + 1].x / mWindowSize.x) * 2.0f;

		if (leftXTime <= mCurrentTime && rightXTime >= mCurrentTime)
		{
			posToLeft = list[i];
			posToRight = list[i + 1];
			break;
		}
	}

	//Laura figured out this math because she is egod of time
	float t = (mCurrentTime - leftXTime) / (rightXTime - leftXTime);
	return egpfwLerp(posToLeft.y, posToRight.y, t) * 2.0f / mWindowSize.y - 1.0f;
}

void KeyframeWindow::renderToFBO(int* curveUniformSet, int* solidColorUniformSet)
{	
	int j;
	cbmath::vec4 *waypointPtr;
	cbmath::mat4 waypointMVP;

	egpfwActivateFBO(mFBOList + curvesFBO);

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

	//Drew all the keyframes. Now, draw the scrub head thing.
	cbmath::vec4 points[2] = { cbmath::vec4(mCurrentTime / 2.0f * mWindowSize.x, 0.0f, 0.0f, 1.0f), cbmath::vec4(mCurrentTime / 2.0f * mWindowSize.x, mWindowSize.y, 0.0f, 1.0f) };
	egpActivateProgram(mProgramList + drawCurveProgram);
	egpSendUniformFloatMatrix(curveUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mLittleBoxWindowMatrix.m);

	egpSendUniformFloat(curveUniformSet[unif_waypoint], UNIF_VEC4, vecSize, points[0].v);
	egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[NUM_OF_CHANNELS].v);
	egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &twoTest);
	egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
	egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

	egpActivateVAO(mVAOList + pointModel);
	egpDrawActiveVAO();

	//Now, draw the x axis.
	points[0] = cbmath::vec4(0.0f, mWindowSize.y / 2.0f, 0.0f, 1.0f);
	points[1] = cbmath::vec4(mWindowSize.x, mWindowSize.y / 2.0f, 0.0f, 1.0f);

	egpActivateProgram(mProgramList + drawCurveProgram);
	egpSendUniformFloatMatrix(curveUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mLittleBoxWindowMatrix.m);

	egpSendUniformFloat(curveUniformSet[unif_waypoint], UNIF_VEC4, vecSize, points[0].v);
	egpSendUniformFloat(solidColorUniformSet[unif_color], UNIF_VEC4, 1, COLORS[NUM_OF_CHANNELS].v);
	egpSendUniformInt(curveUniformSet[unif_waypointCount], UNIF_INT, 1, &twoTest);
	egpSendUniformInt(curveUniformSet[unif_curveMode], UNIF_INT, 1, &zeroTest);
	egpSendUniformInt(curveUniformSet[unif_useWaypoints], UNIF_INT, 1, &trueTest);

	egpActivateVAO(mVAOList + pointModel);
	egpDrawActiveVAO();
	
}

void KeyframeWindow::renderToBackbuffer(int* textureUniformSet)
{
	//Draw our FBO to the backbuffer
	egpActivateProgram(mProgramList + testTextureProgramIndex);
	egpActivateVAO(mVAOList + fsqModel);
	egpfwBindColorTargetTexture(mFBOList + curvesFBO, 0, 0);
	egpSendUniformFloatMatrix(textureUniformSet[unif_mvp], UNIF_MAT4, 1, 0, mOnScreenMatrix.m);
	egpDrawActiveVAO();

	//Draw the axes using immediate mode :( :( :(
	//Seriously though, how do you even do this with the programmable pipeline???

	glBegin(GL_BITMAP); //more like GL_BUTTMAP hahahaha
	egpActivateProgram(0);
	glDisable(GL_TEXTURE_2D);

	auto color = COLORS[mCurrentChannel];
	if (mCurrentChannel == NUM_OF_CHANNELS)
		color = cbmath::vec4(0, 0, 0, 0);

	glColor3f(color.r, color.g, color.b);
	glDisable(GL_LIGHTING);

	//Draw the x axis on top of the box.
	{
		glWindowPos2i(5, mWindowSize.y * 0.41f);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '0');

		glWindowPos2i(mWindowSize.x * 0.4f * 0.5f, mWindowSize.y * 0.41f);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '1');

		glWindowPos2i(mWindowSize.x * 0.4f - 5, mWindowSize.y * 0.41f);
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, '2');
	}

	//Draw the y axis to the side of the box.
	{
		std::string stringToWrite[3];

		//Choose which text to use based on the current channel
		if ((int)mCurrentChannel < (int)CHANNEL_ROT_X)
		{
			stringToWrite[0] = "-5";
			stringToWrite[1] = "0";
			stringToWrite[2] = "5";
		}
		else if ((int)mCurrentChannel < (int)NUM_OF_CHANNELS)
		{
			stringToWrite[0] = "-360";
			stringToWrite[1] = "0";
			stringToWrite[2] = "360";
		}
		else
		{
			return;
		}

		glWindowPos2i(mWindowSize.x * 0.4f + 5, 5);
		for (auto i = 0; i < stringToWrite[0].size(); i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, stringToWrite[0][i]);

		glWindowPos2i(mWindowSize.x * 0.4f + 5, mWindowSize.y * 0.4f * 0.5f);
		for (auto i = 0; i < stringToWrite[1].size(); i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, stringToWrite[1][i]);

		glWindowPos2i(mWindowSize.x * 0.4f + 5, mWindowSize.y * 0.4f - 15);
		for (auto i = 0; i < stringToWrite[2].size(); i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, stringToWrite[2][i]);
	}

	glEnd();
}
