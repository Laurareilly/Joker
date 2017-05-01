// EGPFW Framework Demo - Freeglut main for Windows
// By Dan Buckstein

#include "../../egpfw/TStack.h"
#include "../../egpfw/transformMatrix.h"
#ifdef _WIN32
//-----------------------------------------------------------------------------

#include "../source/egpfwDemo/egpfwDemo.h"

#include "egpfw/egpfw/utils/egpfwInputUtils.h"

// OpenGL & extension wrangler
#include "GL/glew.h"
#include "GL/freeglut.h"

#include <stdio.h>


//-----------------------------------------------------------------------------

unsigned int win_handle;
const char *win_name = "EGP Graphics Demo Window - D. Buckstein";
const unsigned char win_close = 27;	// escape key closes window


//-----------------------------------------------------------------------------
// core functions

// display - refers to the actual display refreshing, not draw time
void onDisplay()
{
	// DO NOT USE
}

void onIdle()
{
	if (idle())
	{
		glutSwapBuffers();
	}
}

// initialize OpenGL using freeglut and GLEW
int initGLUT(int argc, char **argv)
{
	// create a window with double-buffering, RGBA colour and depth buffer
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(windowPosX(), windowPosY());
	glutInitWindowSize(windowWidth(), windowHeight());
	win_handle = glutCreateWindow(win_name);

	// initialize extensions
	int init = glewInit();
	if (init == GLEW_OK)
	{
		// check version again to confirm initialization was successful
		// should be higher than the previous check if default is not highest
		printf("\n GL version: \n");
		printGLVersion();

		// set callbacks
		glutDisplayFunc(onDisplay);
		glutIdleFunc(onIdle);
		glutCloseFunc(onCloseWindow);
		glutReshapeFunc(onResizeWindow);
		glutPositionFunc(onPositionWindow);
		glutMouseFunc(egpMouseButtonCallback);
		glutMotionFunc(egpMouseMotionCallback);
		glutPassiveMotionFunc(egpMouseMotionCallback);
		glutKeyboardFunc(egpKeyboardPressCallback);
		glutKeyboardUpFunc(egpKeyboardReleaseCallback);

		// initialize GL states
		return initGL();
	}

	// fail
	return 0;
}

// close key pressed
void winClosePressed()
{
	glutLeaveMainLoop();
}

// entry function
int main(int argc, char **argv)
{

	// initialize graphics library
	if (initGLUT(argc, argv))
	{
		// initialize image library
		if (initIL())
		{
			// initialize game objects (pre-start)
			if (initGame())
			{
				displayControls();
				///////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////
				//// force first update and render
				updateGameState(0.0f);
				renderGameState();
				///////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////
				//// start timers
				resetTimers();
				///////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////
				//// set close key
				egpSetCloseKey(win_close, winClosePressed);
				///////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////
				//// start loop (launch game)
				glutMainLoop();
				///////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////
			}
		}
	}

	//	printf("\n\n");
	//	system("pause");

	return 0;
}



//-----------------------------------------------------------------------------
#endif	// _WIN32
