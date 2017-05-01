// By Dan Buckstein
// Modified by: _______________________________________________________________
#include "egpfw/egpfw/egpfwFrameBuffer.h"


// OpenGL
#ifdef _WIN32
#include "GL/glew.h"
#else	// !_WIN32
#include <OpenGL/gl3.h>
#endif	// _WIN32


#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// these are the colour targets for OpenGL
const unsigned int egpfwTargetName[] = {
	GL_COLOR_ATTACHMENT0,	GL_COLOR_ATTACHMENT1,	GL_COLOR_ATTACHMENT2,	GL_COLOR_ATTACHMENT3,
	GL_COLOR_ATTACHMENT4,	GL_COLOR_ATTACHMENT5,	GL_COLOR_ATTACHMENT6,	GL_COLOR_ATTACHMENT7,
	GL_COLOR_ATTACHMENT8,	GL_COLOR_ATTACHMENT9,	GL_COLOR_ATTACHMENT10,	GL_COLOR_ATTACHMENT11,
	GL_COLOR_ATTACHMENT12,	GL_COLOR_ATTACHMENT13,	GL_COLOR_ATTACHMENT14,	GL_COLOR_ATTACHMENT15,
};

// color formats
const unsigned int egpfwInternalColorFormat[] = {
	0, GL_RGB8, GL_RGB16, GL_RGB32F, GL_RGBA8, GL_RGBA16, GL_RGBA32F,
};

// color storage type
const unsigned int egpfwInternalColorStorage[] = {
	0, GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, GL_FLOAT, GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, GL_FLOAT
};

// depth formats
const unsigned int egpfwInternalDepthFormat[] = {
	0, GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT32, GL_DEPTH24_STENCIL8,
};

// depth storage type
const unsigned int egpfwInternalDepthStorage[] = {
	0, GL_UNSIGNED_SHORT, GL_UNSIGNED_INT, GL_UNSIGNED_INT, GL_UNSIGNED_INT
};


//-----------------------------------------------------------------------------

// ****
egpFrameBufferObjectDescriptor egpfwCreateFBO(const unsigned int frameWidth, const unsigned int frameHeight, const unsigned int numColorTargets, const egpColorFormat colorFormat, const egpDepthFormat depthFormat, const egpWrapSmoothFormat wrapSmoothFormat)
{
  egpFrameBufferObjectDescriptor fbo = { 0 };

  unsigned int format, internalFormat, internalStorage, attachmentType;
  unsigned int hWrap, vWrap, smooth;

  hWrap = (wrapSmoothFormat == WRAP_HORIZ || wrapSmoothFormat == WRAP_HORIZ_VERT ||
    wrapSmoothFormat == SMOOTH_WRAP_H || wrapSmoothFormat == SMOOTH_WRAP) ? GL_REPEAT : GL_CLAMP_TO_EDGE;
  vWrap = (wrapSmoothFormat == WRAP_VERT || wrapSmoothFormat == WRAP_HORIZ_VERT ||
    wrapSmoothFormat == SMOOTH_WRAP_V || wrapSmoothFormat == SMOOTH_WRAP) ? GL_REPEAT : GL_CLAMP_TO_EDGE;
  smooth = (wrapSmoothFormat >= SMOOTH_NOWRAP) ? GL_LINEAR : GL_NEAREST;

  glGenFramebuffers(1, &fbo.glhandle);
  if (fbo.glhandle) {
    fbo.frameWidth = frameWidth;
    fbo.frameHeight = frameHeight;
    fbo.numColorTargets = numColorTargets;
    fbo.depthFormat = depthFormat;
    fbo.wrapSmoothFormat = wrapSmoothFormat;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo.glhandle);

    format = GL_RGBA;
    internalFormat = egpfwInternalColorFormat[colorFormat];
    internalStorage = egpfwInternalColorStorage[colorFormat];

    glGenTextures(fbo.numColorTargets, fbo.colorTargetHandle);
    for (unsigned int i = 0; i < fbo.numColorTargets; ++i) {
      glBindTexture(GL_TEXTURE_2D, fbo.colorTargetHandle[i]);
      glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, frameWidth, frameHeight, 0, format, internalStorage, 0);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, smooth);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, smooth);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, hWrap);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, vWrap);

      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_TEXTURE_2D, fbo.colorTargetHandle[i], 0);
    }

    if (depthFormat != DEPTH_DISABLE) {

      fbo.hasDepthTarget = 1;
      fbo.hasStencilTarget = (depthFormat == DEPTH_D24S8);
      format = fbo.hasStencilTarget ? GL_DEPTH_STENCIL : GL_DEPTH_COMPONENT;

      internalFormat = egpfwInternalDepthFormat[depthFormat];
      internalStorage = egpfwInternalDepthStorage[depthFormat];
      attachmentType = fbo.hasStencilTarget ? GL_DEPTH_STENCIL_ATTACHMENT : GL_DEPTH_ATTACHMENT;

      glGenTextures(1, &fbo.depthTargetHandle[0]);
      glBindTexture(GL_TEXTURE_2D, fbo.depthTargetHandle[0]);
      glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, frameWidth, frameHeight, 0, format, internalStorage, 0);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, smooth);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, smooth);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, hWrap);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, vWrap);


      glFramebufferTexture2D(GL_FRAMEBUFFER, attachmentType, GL_TEXTURE_2D, fbo.depthTargetHandle[0], 0);
    }

    if (!glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
      // something went wrong, release
      printf("\n FBO creation failed! Validation failed, FBO deleted.");
      egpfwReleaseFBO(&fbo);
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  return fbo;
}


// ****
void egpfwActivateFBO(const egpFrameBufferObjectDescriptor *fbo)
{

  if (!fbo || !fbo->glhandle) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, fbo->glhandle);
  if (fbo->numColorTargets) {
    glDrawBuffers(fbo->numColorTargets, egpfwTargetName);
  }

  if (fbo->hasDepthTarget) {
    glEnable(GL_DEPTH_TEST);
    if (fbo->hasStencilTarget) {
      glEnable(GL_STENCIL_TEST);
    }
  } else {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
  }


  glViewport(0, 0, fbo->frameWidth, fbo->frameHeight);

}


// ****
int egpfwReleaseFBO(egpFrameBufferObjectDescriptor *fbo)
{
	//...
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  glDeleteFramebuffers(1, &fbo->glhandle);

  if (fbo->numColorTargets) {
    glDeleteTextures(fbo->numColorTargets, fbo->colorTargetHandle);
  }

  if (fbo->hasDepthTarget) {
    glDeleteTextures(1, fbo->depthTargetHandle);
  }

	return 0;
}


// ****
int egpfwBindColorTargetTexture(const egpFrameBufferObjectDescriptor *fbo, const unsigned int glBinding, const unsigned int targetIndex)
{
  if (fbo && fbo->numColorTargets && targetIndex < 16) {
    glActiveTexture(GL_TEXTURE0 + glBinding);
    glBindTexture(GL_TEXTURE_2D, fbo->colorTargetHandle[targetIndex]);
    return 1;
  }
	return 0;
}

// ****
int egpfwBindDepthTargetTexture(const egpFrameBufferObjectDescriptor *fbo, const unsigned int glBinding)
{
  if (fbo && fbo->hasDepthTarget) {
    glActiveTexture(GL_TEXTURE0 + glBinding);
    glBindTexture(GL_TEXTURE_2D, fbo->depthTargetHandle[0]);
    return 1;
  }
  return 0;
}
