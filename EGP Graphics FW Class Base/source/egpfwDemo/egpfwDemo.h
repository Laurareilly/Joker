///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////	EGP Graphics Framework Demo
////	By Dan Buckstein
////	January 2017
////
////	****Modified by: ______________________________________________________
////	^^^^
////	NOTE: 4 asterisks anywhere means something you will have to either 
////		modify or complete to get it working! Good practice for starters: 
////		write your name beside "modified by" above!
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#ifndef __EGPFW_DEMO_H
#define __EGPFW_DEMO_H


#ifdef __cplusplus
extern "C"
{
#endif  // __cplusplus


	unsigned int windowWidth();
	unsigned int windowHeight();
	unsigned int windowPosX();
	unsigned int windowPosY();

	int initGL();
	int initIL();
	int initGame();
	void printGLVersion();

	void displayControls();
	void updateGameState(float dt);
	void renderGameState();
	void handleInputState();
	void resetTimers();

	void onCloseWindow();
	void onResizeWindow(int w, int h);
	void onPositionWindow(int x, int y);

	int idle();


#ifdef __cplusplus
}
#endif  // __cplusplus


#endif	// __EGPFW_DEMO_H
