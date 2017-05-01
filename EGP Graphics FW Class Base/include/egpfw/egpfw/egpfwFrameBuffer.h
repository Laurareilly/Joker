/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	Frame buffer utilities by Dan Buckstein

	Modified by: ______________________________________________________________
*/

#ifndef __EGPFW_FRAMEBUFFER_H
#define __EGPFW_FRAMEBUFFER_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


//-----------------------------------------------------------------------------
// enumerators

	// color buffer format
	enum egpColorFormat
	{
		COLOR_DISABLE,		// no color targets (yes, this is allowed)
		COLOR_RGB8,			// 3 channels, 8 bits each (internally is a byte)
		COLOR_RGB16,		// 3 channels, 16 bits each (short)
		COLOR_RGB32F,		// 3 channels, 32 bits each (float, use sparingly)
		COLOR_RGBA8,		// 4 channels, 8 bits (byte)
		COLOR_RGBA16,		// 4 channels, 16 bits (short)
		COLOR_RGBA32F,		// 4 channels, 32 bits (float, use sparingly)
	};

	// depth buffer format
	enum egpDepthFormat
	{
		DEPTH_DISABLE,		// no depth target
		DEPTH_D16,			// 1 channel, 16 bits (65536 possible values)
		DEPTH_D24,			// 1 channel, 24 bits (16 million possible values)
		DEPTH_D32,			// 1 channel, 32 bits (4 billion... max precision)
		DEPTH_D24S8,		// depth buffer uses 24 bits, stencil 8 bits
	};

	// texture wrapping and smoothing option
	enum egpWrapSmoothFormat
	{
		WRAP_DISABLE,		// clamps on both dimensions, no smoothing
		WRAP_HORIZ,			// textures wrap horizontally, vertical clamps
		WRAP_VERT,			// wrap vertically, clamp horizontally
		WRAP_HORIZ_VERT,	// wrap both horiz and vert
		SMOOTH_NOWRAP,		// pixels are smoothed by linear interpolation
		SMOOTH_WRAP_H,		// smoothed pixels and horizontal wrapping
		SMOOTH_WRAP_V,		// smoothed pixels and vertical wrapping
		SMOOTH_WRAP,		// smoothed pixels and full wrapping
	};

#ifndef __cplusplus
	typedef enum egpColorFormat			egpColorFormat;
	typedef enum egpDepthFormat			egpDepthFormat;
	typedef enum egpWrapSmoothFormat	egpWrapSmoothFormat;
#endif	// __cplusplus
	

//-----------------------------------------------------------------------------
// data structure

#ifndef __cplusplus
	typedef struct egpFrameBufferObjectDescriptor egpFrameBufferObjectDescriptor;
#endif	// __cplusplus

	// OpenGL framebuffer object (FBO)
	// contains handle to internal FBO and information about its targets
	// additionally, has a bunch of texture handles
	// also contains formats as per the above enums
	struct egpFrameBufferObjectDescriptor
	{
		unsigned int glhandle;
		unsigned int frameWidth, frameHeight;
		unsigned int numColorTargets, hasDepthTarget, hasStencilTarget;
		unsigned int colorTargetHandle[16], depthTargetHandle[1];
		egpColorFormat colorFormat;
		egpDepthFormat depthFormat;
		egpWrapSmoothFormat wrapSmoothFormat;
	};


//-----------------------------------------------------------------------------
// FBO functions

	// generate a framebuffer object
	// also generates a set of textures for render targets
	// think of it as a sketchpad with multiple pages
	// 'frameWidth' and 'frameHeight' params must be greater than zero
	// if color format param is 'disable' or num color targets param is zero, 
	//	generates FBO with no color targets
	// 'numColorTargets' param must be no more than 16
	// if depth format param is 'disable', generates FBO with no depth target
	// the total number of targets must not be zero or nothing will be created
	egpFrameBufferObjectDescriptor egpfwCreateFBO(const unsigned int frameWidth, const unsigned int frameHeight, const unsigned int numColorTargets, const egpColorFormat colorFormat, const egpDepthFormat depthFormat, const egpWrapSmoothFormat wrapSmoothFormat);

	// bind a framebuffer for drawing
	// 'fbo' param can be null to deactivate FBO
	void egpfwActivateFBO(const egpFrameBufferObjectDescriptor *fbo);

	// delete a framebuffer internally
	// returns 1 if success, 0 if failed
	int egpfwReleaseFBO(egpFrameBufferObjectDescriptor *fbo);

	// activate a rendered frame as a texture to be used in another pass
	// depth target includes stencil target if one is attached
	// 'fbo' param cannot be null
	// 'glBinding' represents the target "outlet" that the texture should be 
	//	"plugged into" for use, must be less than 16
	// 'targetIndex' must be less than 16
	// function will return 1 if the call is valid, 0 if invalid
	int egpfwBindColorTargetTexture(const egpFrameBufferObjectDescriptor *fbo, const unsigned int glBinding, const unsigned int targetIndex);
	int egpfwBindDepthTargetTexture(const egpFrameBufferObjectDescriptor *fbo, const unsigned int glBinding);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_FRAMEBUFFER_H