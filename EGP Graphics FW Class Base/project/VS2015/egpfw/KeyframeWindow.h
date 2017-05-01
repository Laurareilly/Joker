#pragma once
#include "egpfw/egpfw.h"

#include <vector>
#include <array>
#include <cbmath/cbtkVector.h>
#include <cbmath/cbtkMatrix.h>

struct egpMouse;
struct egpKeyboard;

class KeyframeWindow
{
	public:
		enum KeyframeChannel
		{
			CHANNEL_POS_X = 0,
			CHANNEL_POS_Y,
			CHANNEL_POS_Z,
			CHANNEL_ROT_X,
			CHANNEL_ROT_Y,
			CHANNEL_ROT_Z,
			NUM_OF_CHANNELS,
		};

	private:
		egpVertexArrayObjectDescriptor* mVAOList;
		egpFrameBufferObjectDescriptor* mFBOList;
		egpProgram* mProgramList;

		std::array<std::vector<cbmath::vec4>, NUM_OF_CHANNELS> mWaypointChannels;
		KeyframeChannel mCurrentChannel;
		cbmath::vec2 mWindowSize;
		cbmath::mat4 mLittleBoxWindowMatrix;
		cbmath::mat4 mOnScreenMatrix, mOnScreenMatrixInv;
		float mCurrentTime;
		bool mIsPaused;

		void resetKeyframes();

	public:
		KeyframeWindow(egpVertexArrayObjectDescriptor* vao, egpFrameBufferObjectDescriptor* fbo, egpProgram* programs);
		~KeyframeWindow();

		//

		/**
		 * \brief 
		 * \return True if the mouse is currently inside of the bounds of the window. False otherwise. */
		bool updateInput(egpMouse* m, egpKeyboard* key);
		void update(float deltaT);
		void updateWindowSize(float viewport_tw, float viewport_th, float tmpNF, float win_w, float win_h);

		float getValAtCurrentTime(KeyframeChannel c);
	
		cbmath::mat4& getOnScreenMatrix() { return mOnScreenMatrix; }

		void renderToFBO(int* curveUniformSet, int* solidColorUniformSet);
		void renderToBackbuffer(int* textureUniformSet);
};

