/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Mouse & keyboard input control utility by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_INPUT_H
#define __EGPFW_UTILS_INPUT_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// data structures

	// mouse
	struct egpMouse
	{
		// current cursor location and change
		int p[2], p0[2];

		// button states
		// button IDs are 0 for left, 1 for middle and 2 for right
		char btn[4], btn0[4];
	};


	// keyboard
	struct egpKeyboard
	{
		// key states
		char key[256], key0[256];
	};

	// special keys
	enum egpKeySpecial
	{
		KEY_LEFT, 
		KEY_UP, 
		KEY_RIGHT, 
		KEY_DOWN, 
	};


#ifndef __cplusplus
	typedef struct egpMouse		egpMouse;
	typedef struct egpKeyboard	egpKeyboard;
#endif	// !__cplusplus


//-----------------------------------------------------------------------------
// functions

	// tell mouse that a button has been pressed or released
	// 'mouse' param cannot be null
	void egpMousePressButton(egpMouse *mouse, const unsigned char btn);
	void egpMouseReleaseButton(egpMouse *mouse, const unsigned char btn);

	// set mouse position using screen X, Y
	void egpMouseSetPosition(egpMouse *mouse, const int x, const int y);

	// update mouse states
	// 'mouse' param cannot be null
	void egpMouseUpdate(egpMouse *mouse);

	// reset mouse
	// 'mouse' param cannot be null
	void egpMouseReset(egpMouse *mouse);

	// query mouse states
	// all return 1 for yes and 0 for no
	// 'mouse' param cannot be null
	int egpMouseIsButtonDown(egpMouse *mouse, const unsigned char btn);
	int egpMouseIsButtonUp(egpMouse *mouse, const unsigned char btn);
	int egpMouseIsButtonPressed(egpMouse *mouse, const unsigned char btn);
	int egpMouseIsButtonReleased(egpMouse *mouse, const unsigned char btn);

	// query mouse position
	// 'mouse' param cannot be null
	int egpMouseX(egpMouse *mouse);
	int egpMouseY(egpMouse *mouse);
	int egpMouseDeltaX(egpMouse *mouse);
	int egpMouseDeltaY(egpMouse *mouse);


	// tell keyboard that a key has been pressed or released
	// key input is the character for that key
	// 'keybd' param cannot be null
	void egpKeyboardPressKey(egpKeyboard *keybd, const unsigned char key);
	void egpKeyboardReleaseKey(egpKeyboard *keybd, const unsigned char key);

	// update keyboard states
	// 'keybd' param cannot be null
	void egpKeyboardUpdate(egpKeyboard *keybd);

	// reset keyboard
	// 'keybd' param cannot be null
	void egpKeyboardReset(egpKeyboard *keybd);

	// query keyboard states
	// all return 1 for yes and 0 for no
	// 'keybd' param cannot be null
	int egpKeyboardIsKeyDown(egpKeyboard *keybd, const unsigned char key);
	int egpKeyboardIsKeyUp(egpKeyboard *keybd, const unsigned char key);
	int egpKeyboardIsKeyPressed(egpKeyboard *keybd, const unsigned char key);
	int egpKeyboardIsKeyReleased(egpKeyboard *keybd, const unsigned char key);

	// utility to check the difference between a key that should 
	//	represent a negative value and one that should represent 
	//	a positive value (e.g. wasd: forward = w - s)
	// 'keybd' param cannot be null
	int egpKeyboardDifference(egpKeyboard *keybd, const unsigned char positive, const unsigned char negative);


//-----------------------------------------------------------------------------
// OpenGL callbacks that wire straight up to the above structures and functions

	// set active mouse and keyboard internally
	// callbacks below will not work if this is not done
	void egpSetActiveMouse(egpMouse *mouse);
	void egpSetActiveKeyboard(egpKeyboard *keybd);

	// utility to close the window when a specific key is pressed
	void egpSetCloseKey(unsigned char key, void(*closeCallback)());

	// mouse callbacks
	void egpMouseButtonCallback(int, int, int, int);
	void egpMouseMotionCallback(int, int);

	// keyboard callbacks
	void egpKeyboardPressCallback(unsigned char, int, int);
	void egpKeyboardReleaseCallback(unsigned char, int, int);
	void egpKeyboardSpecialPressCallback(int, int, int);
	void egpKeyboardSpecialReleaseCallback(int, int, int);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_INPUT_H