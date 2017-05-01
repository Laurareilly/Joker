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

#include "egpfwDemo.h"

//-----------------------------------------------------------------------------
// external includes

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// third party math lib
#include "cbmath/cbtkMatrix.h"

// OpenGL
#ifdef _WIN32
#include "GL/glew.h"
#else	// !_WIN32
#include <OpenGL/gl3.h>
#endif	// _WIN32

// DevIL (images)
#include "IL/ilut.h"


//-----------------------------------------------------------------------------
// framework includes

#include "egpfw/egpfw.h"
#include "../../project/VS2015/egpfw/RenderPath.h"
#include "../../project/VS2015/egpfw/RenderNetgraph.h"
#include "../../project/VS2015/egpfw/KeyframeWindow.h"
#include <GL/freeglut.h>


//-----------------------------------------------------------------------------
// globals


//-----------------------------------------------------------------------------

// window info
unsigned int win_x = 0;
unsigned int win_y = 0;
unsigned int win_w = 1280;
unsigned int win_h = 720;
const unsigned int viewport_b = 8;
const unsigned int viewport_tb = viewport_b + viewport_b;
const int viewport_nb = -(int)viewport_b;
unsigned int viewport_tw = win_w + viewport_tb;
unsigned int viewport_th = win_h + viewport_tb;
float win_aspect;
float fovy = 1.0f, znear = 0.5f, zfar = 50.0f;
float maxClipDist = 0.0f, minClipDist = 0.0f;

// update flag: play speed as a percentage
unsigned int playing = 1;
unsigned int playrate = 100;


// timers
egpTimer renderTimer[1], secondTimer[1];

// controls
egpMouse mouse[1];
egpKeyboard keybd[1];


// general: camera's view matrix and projection matrix
cbtk::cbmath::mat4 viewMatrix, projectionMatrix, viewProjMat;
// camera controls
float cameraElevation = 0.25f, cameraAzimuth = 4.0f;
float cameraRotateSpeed = 0.25f, cameraMoveSpeed = 4.0f, cameraDistance = 8.0f;
cbtk::cbmath::vec4 cameraPosWorld(1.0f, -2.0f, -cameraDistance, 1.0f), deltaCamPos;

float CONST_ZERO_FLOAT = 0.0f;

enum RenderMethod
{
	bloomRenderMethod = 0,
	deferredRenderMethod,
	depthOfFieldRenderMethod,

	//------------------------
	numRenderMethods
};

RenderMethod& operator++(RenderMethod& a)
{
	int tmp = a;
	if (tmp == (numRenderMethods - 1))
		tmp = -1;

	a = static_cast<RenderMethod>(tmp + 1);
	return a;
}

RenderMethod currentRenderMode = bloomRenderMethod;
bool displayNetgraphToggle = true;

//-----------------------------------------------------------------------------
// graphics-related data and handles
// good practice: default values for everything

egpVertexArrayObjectDescriptor vao[modelCount] = { 0 };
egpVertexBufferObjectDescriptor vbo[modelCount] = { 0 };


// loaded textures
enum TextureIndex
{
	skyboxTexHandle,
	earthTexHandle_dm, earthTexHandle_sm,
	moonTexHandle_dm,

	atlas_diffuse,
	atlas_specular,

	//-----------------------------
	textureCount
};
unsigned int tex[textureCount] = { 0 };

egpProgram glslPrograms[GLSLProgramCount] = { 0 }, *currentProgram = 0;
int glslCommonUniforms[GLSLProgramCount][GLSLCommonUniformCount] = { -1 }, *currentUniformSet = 0;
int currentProgramIndex = 0;

egpFrameBufferObjectDescriptor fbo[fboCount], *fboFinalDisplay = fbo;
cbmath::vec2 pixelSizeInv[fboCount];
int testDrawAxes = 0;
int displayMode = compositeFBO;



//-----------------------------------------------------------------------------
// our game objects

// transformation matrices
cbmath::mat4 skyboxModelViewMatrix, skyboxModelViewProjectionMatrix;
cbmath::mat4 earthModelMatrix, earthModelViewProjectionMatrix, earthModelInverseMatrix;
cbmath::mat4 moonModelMatrix, moonModelViewProjectionMatrix, moonModelInverseMatrix;

// transformation matrices
cbmath::mat4 skyboxModelMatrix, skyboxAtlasMatrix;
cbmath::mat4 earthAtlasMatrix;
cbmath::mat4 moonAtlasMatrix;
cbmath::mat4 marsModelMatrix, marsAtlasMatrix;
cbmath::mat4 groundModelMatrix, groundAtlasMatrix;

int curveMode;
const int waypoint_max = 16;
int waypointCount = 0;
int useWaypoints = 1;
cbmath::vec4 waypoint[waypoint_max];
cbmath::mat4 curveDrawingProjectionMatrix, waypointModelMatrix = cbmath::makeScale4(4.0f);

// light positions and colors
// for the colors, let the xyz components represent color in rgb, and 
//	the w component represent the radius of the light volume
const unsigned int numLights = 16, numLightsShading = 4;
cbmath::vec4 lightPos_world[numLights] = {
	cbmath::vec4(0.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(10.0f, 0.0f, 0.0f, 1.0f),
	cbmath::vec4(0.0f, 10.0f, 10.0f, 1.0f),
	cbmath::vec4(50.0f, 50.0f, 50.0f, 1.0f),

	cbmath::vec4(-6.0f, -4.0f, -3.0f, 1.0f),
	cbmath::vec4(-2.0f, -4.0f, -3.0f, 1.0f),
	cbmath::vec4(+2.0f, -4.0f, -3.0f, 1.0f),
	cbmath::vec4(+6.0f, -4.0f, -3.0f, 1.0f),

	cbmath::vec4(-6.0f, -4.0f, 0.0f, 1.0f),
	cbmath::vec4(-2.0f, -4.0f, 0.0f, 1.0f),
	cbmath::vec4(+2.0f, -4.0f, 0.0f, 1.0f),
	cbmath::vec4(+6.0f, -4.0f, 0.0f, 1.0f),

	cbmath::vec4(-6.0f, -4.0f, +3.0f, 1.0f),
	cbmath::vec4(-2.0f, -4.0f, +3.0f, 1.0f),
	cbmath::vec4(+2.0f, -4.0f, +3.0f, 1.0f),
	cbmath::vec4(+6.0f, -4.0f, +3.0f, 1.0f),
};
cbmath::vec4 lightColor[numLights] = {
	cbmath::vec4(0.0f, 1.0f, 1.0f, 10.0f),
	cbmath::vec4(1.0f, 1.0f, 0.0f, 10.0f),
	cbmath::vec4(1.0f, 0.0f, 1.0f, 20.0f),
	cbmath::vec4(1.0f, 1.0f, 1.0f, 20.0f),

	cbmath::vec4(1.0f, 0.0f, 0.0f, 2.0f),
	cbmath::vec4(1.0f, 0.5f, 0.0f, 2.0f),
	cbmath::vec4(1.0f, 1.0f, 0.0f, 2.0f),
	cbmath::vec4(0.5f, 1.0f, 0.0f, 2.0f),

	cbmath::vec4(0.0f, 1.0f, 0.0f, 2.0f),
	cbmath::vec4(0.0f, 1.0f, 0.5f, 2.0f),
	cbmath::vec4(0.0f, 1.0f, 1.0f, 2.0f),
	cbmath::vec4(0.0f, 0.5f, 1.0f, 2.0f),

	cbmath::vec4(0.0f, 0.0f, 1.0f, 2.0f),
	cbmath::vec4(0.5f, 0.0f, 1.0f, 2.0f),
	cbmath::vec4(1.0f, 0.0f, 1.0f, 2.0f),
	cbmath::vec4(1.0f, 0.0f, 0.5f, 2.0f),
};



// light and camera for shading
cbmath::vec4 lightPos_object, eyePos_object;


// raw animation values: 
float earthDaytime = 0.0f;
float earthOrbit = 0.0f;
const float earthAnimateRate = 100.0f;
const float earthDaytimePeriod = earthAnimateRate / 24.0f;
const float earthOrbitPeriod = earthDaytimePeriod / 365.25f;
const float earthTilt = Deg2Rad(23.44f);
const float earthDistance = 6.0f;

float moonOrbit = 0.0f;
const float moonOrbitPeriod = earthDaytimePeriod / 27.32f;
const float moonTilt = Deg2Rad(6.69f);
const float moonDistance = 4.0f;
const float moonSize = 0.27f;

RenderPath globalRenderPath;
RenderNetgraph globalRenderNetgraph(fbo, vao + fsqModel, glslPrograms, glslCommonUniforms[testTextureProgramIndex]);

KeyframeWindow keyframeWindow(vao, fbo, glslPrograms);


//-----------------------------------------------------------------------------
// game functions

// access window info
unsigned int windowWidth()
{
	return win_w;
}

unsigned int windowHeight()
{
	return win_h;
}

unsigned int windowPosX()
{
	return win_x;
}

unsigned int windowPosY()
{
	return win_y;
}


// set initial GL states
int initGL()
{
	const float lineWidth = 4.0f;
	const float pointSize = lineWidth;

	// textures ON
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);

	// backface culling ON
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	// depth testing OFF
	glDisable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	// alpha blending OFF
	// result = ( new*[new alpha] ) + ( old*[1 - new alpha] )
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// lines and points
	glLineWidth(lineWidth);
	glPointSize(pointSize);

	// done
	return 1;
}


// initialize third-party image handling using DevIL
int initIL()
{
	ilInit();
	iluInit();
	ilutInit();
	ilutRenderer(ILUT_OPENGL);

	// done
	return 1;
}


// display OpenGL version
void printGLVersion()
{
	printf("%s", glGetString(GL_VERSION));
}



//-----------------------------------------------------------------------------

// load and delete geometry
void setupGeometry()
{
	// prepare sample geometry
	// all of these are statically defined, so there is no cleanup for them
	egpInitializeCube();
	egpInitializeCubeIndexed();
	egpInitializeWireCube();
	egpInitializeWireCubeIndexed();
	egpInitializeSphere8x6();
	egpInitializeSphere32x24();


	// prepare sample geometry using VBOs
	// these should be cleaned

	// good news is they all use the same attribute descriptors, 
	//	so prepare those first
	egpAttributeDescriptor attribs[] = {
		egpCreateAttributeDescriptor(ATTRIB_POSITION, ATTRIB_VEC3, 0),
		egpCreateAttributeDescriptor(ATTRIB_COLOR, ATTRIB_VEC3, 0),
		egpCreateAttributeDescriptor(ATTRIB_NORMAL, ATTRIB_VEC3, 0),
		egpCreateAttributeDescriptor(ATTRIB_TEXCOORD, ATTRIB_VEC2, 0),
	};

	// axes
	attribs[0].data = egpGetAxesPositions();
	attribs[1].data = egpGetAxesColors();
	vao[axesModel] = egpCreateVAOInterleaved(PRIM_LINES, attribs, 2, egpGetAxesVertexCount(), (vbo + axesModel), 0);

	// skybox
	attribs[0].data = egpGetCubePositions();
	attribs[1].data = egpGetCubeColors();
	attribs[2].data = egpGetCubeNormals();
	attribs[3].data = egpGetCubeTexcoords();
	vao[skyboxModel] = egpCreateVAOInterleaved(PRIM_TRIANGLES, attribs, 4, egpGetCubeVertexCount(), (vbo + skyboxModel), 0);

	// low-res sphere
	attribs[0].data = egpGetSphere8x6Positions();
	attribs[1].data = egpGetSphere8x6Colors();
	attribs[2].data = egpGetSphere8x6Normals();
	attribs[3].data = egpGetSphere8x6Texcoords();
	vao[sphere8x6Model] = egpCreateVAOInterleaved(PRIM_TRIANGLES, attribs, 4, egpGetSphere8x6VertexCount(), (vbo + sphere8x6Model), 0);

	// high-res sphere
	attribs[0].data = egpGetSphere32x24Positions();
	attribs[1].data = egpGetSphere32x24Colors();
	attribs[2].data = egpGetSphere32x24Normals();
	attribs[3].data = egpGetSphere32x24Texcoords();
	vao[sphere32x24Model] = egpCreateVAOInterleaved(PRIM_TRIANGLES, attribs, 4, egpGetSphere32x24VertexCount(), (vbo + sphere32x24Model), 0);

	// full-screen quad (positions and texcoords only!)
	attribs[1] = attribs[3];
	attribs[0].data = egpfwGetUnitQuadPositions();
	attribs[1].data = egpfwGetUnitQuadTexcoords();
	vao[fsqModel] = egpCreateVAOInterleaved(PRIM_TRIANGLE_STRIP, attribs, 2, 4, (vbo + fsqModel), 0);

	attribs[0].data = cbmath::v3zero.v;
	vao[pointModel] = egpCreateVAOInterleaved(PRIM_POINT, attribs, 1, 1, (vbo + pointModel), 0);

	// loaded models
	// attempt to load binary first; if failed, load object and save binary
	// binary save/load is not necessary, but it is very fast
	egpTriOBJDescriptor obj[1];

	// low-res sphere
	*obj = egpfwLoadBinaryOBJ("sphere8x6_bin.txt");
	if (!obj->data)
	{
		*obj = egpfwLoadTriangleOBJ("../../../../resource/obj/sphere8x6.obj", NORMAL_LOAD, 1.0);
		egpfwSaveBinaryOBJ(obj, "sphere8x6_bin.txt");
	}
	egpfwCreateVAOFromOBJ(obj, vao + sphereLowResObjModel, vbo + sphereLowResObjModel);
	egpfwReleaseOBJ(obj);

	// high-res sphere
	*obj = egpfwLoadBinaryOBJ("sphere32x24_bin.txt");
	if (!obj->data)
	{
		*obj = egpfwLoadTriangleOBJ("../../../../resource/obj/sphere32x24.obj", NORMAL_LOAD, 1.0);
		egpfwSaveBinaryOBJ(obj, "sphere32x24_bin.txt");
	}
	egpfwCreateVAOFromOBJ(obj, vao + sphereHiResObjModel, vbo + sphereHiResObjModel);
	egpfwReleaseOBJ(obj);

	// geometry-related constants
	groundModelMatrix = cbmath::makeTranslation4(0.0f, -5.0f, 0.0f) * cbmath::makeRotationX4(Deg2Rad(-90.0f)) * cbmath::makeScale4(10.0f, 10.0f, 1.0f);

	skyboxAtlasMatrix = groundAtlasMatrix = cbmath::makeScale4(0.125f, 0.125f, 0.0f);
	earthAtlasMatrix = moonAtlasMatrix = marsAtlasMatrix = cbmath::makeScale4(0.5f, 0.25f, 0.0f);
	earthAtlasMatrix.c3.y = 0.75f;
	moonAtlasMatrix.c3.y = 0.5f;
	marsAtlasMatrix.c3.y = 0.25f;
	groundAtlasMatrix.c3.y = 0.125f;

}

void deleteGeometry()
{
	unsigned int i;
	for (i = 0; i < modelCount; ++i)
	{
		egpReleaseVAO(vao + i);
		egpReleaseVBO(vbo + i);
	}
}


// setup and delete textures
void setupTextures()
{
	// here we can change the texture settings: 
	//	1. smoothing: do we want it to appear 
	//		rough/pixelated (GL_NEAREST) or smooth (GL_LINEAR)?
	//	2. do we want it to repeat (GL_REPEAT) 
	//		or clamp to the edges (GL_CLAMP) if texcoords are 
	//		less than 0 or greater than 1?

	// files
	char *imgFiles[] = {
		(char *)("../../../../resource/tex/bg/sky_clouds.png"),
		(char *)("../../../../resource/tex/earth/2k/earth_dm_2k.png"),
		(char *)("../../../../resource/tex/earth/2k/earth_sm_2k.png"),
		(char *)("../../../../resource/tex/moon/2k/moon_dm_2k.png"),
		(char *)("../../../../resource/tex/atlas/atlas_diffuse.png"),
		(char *)("../../../../resource/tex/atlas/atlas_specular.png"),
	};

	// load
	unsigned int ilHandle[textureCount] = { 0 }, status = 0, imgW = 0, imgH = 0;
	const unsigned char *data = 0;
	ilGenImages(textureCount, ilHandle);
	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
	glGenTextures(textureCount, tex);
	for (unsigned int i = 0; i < textureCount; ++i)
	{
		ilBindImage(ilHandle[i]);
		status = ilLoadImage(imgFiles[i]);
		if (status)
		{
			ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);
			imgW = ilGetInteger(IL_IMAGE_WIDTH);
			imgH = ilGetInteger(IL_IMAGE_HEIGHT);
			data = ilGetData();

			glBindTexture(GL_TEXTURE_2D, tex[i]);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, imgW, imgH, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
		}
	}
	ilDeleteImages(textureCount, ilHandle);


	// skybox
	glBindTexture(GL_TEXTURE_2D, tex[skyboxTexHandle]);					// activate 2D texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	// texture gets small/large, smooth
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);		// texture repeats on horiz axis
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);


	// earth textures
	glBindTexture(GL_TEXTURE_2D, tex[earthTexHandle_dm]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);		// these two are deliberately different
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, tex[earthTexHandle_sm]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


	// moon textures
	glBindTexture(GL_TEXTURE_2D, tex[moonTexHandle_dm]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// atlas textures
	glBindTexture(GL_TEXTURE_2D, tex[atlas_diffuse]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);	// pixelated for demonstration purposes
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, tex[atlas_specular]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// disable textures
	glBindTexture(GL_TEXTURE_2D, 0);
}

void deleteTextures()
{
	// delete all textures at once
	glDeleteTextures(textureCount, tex);

}

// array of common uniform names
const char *commonUniformName[] = {
	(const char *)("mvp"),
	(const char *)("lightColor"),
	(const char *)("lightPos"),
	(const char *)("eyePos"),
	(const char *)("tex_dm"),
	(const char *)("tex_sm"),
	(const char *)("pixelSizeInv"),
	(const char *)("img"),
	(const char *)("img1"),
	(const char *)("img2"),
	(const char *)("img3"),
	(const char *)("img4"),
	(const char *)("modelMat"),
	(const char *)("viewprojMat"),
	(const char *)("atlasMat"),
	(const char *)("normalScale"),
	(const char *)("img_position"),
	(const char *)("img_normal"),
	(const char *)("img_texcoord"),
	(const char *)("img_depth"),
	(const char *)("img_light_diffuse"),
	(const char *)("img_light_specular"),
	(const char*)("waypoint"),
	(const char*)("waypointCount"),
	(const char*)("curveMode"),
	(const char*)("useWaypoints"),
	(const char *)("color"),
};

// setup and delete shaders
void setupShaders()
{
	unsigned int u = 0;
	egpFileInfo files[4];
	egpShader shaders[4];


	const int imageLocations[] = {
		0, 1, 2, 3, 4, 5, 6, 7
	};


	// activate a VAO for validation
	egpActivateVAO(vao + sphere8x6Model);


	// test color program
	{
		// load files
		files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs/passColor_vs4x.glsl");
		files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs/drawColor_fs4x.glsl");

		// create shaders
		shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);
		shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

		currentProgramIndex = testColorProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;

		// create, link and validate program
		*currentProgram = egpCreateProgram();
		egpAttachShaderToProgram(currentProgram, shaders + 0);
		egpAttachShaderToProgram(currentProgram, shaders + 1);
		egpLinkProgram(currentProgram);
		egpValidateProgram(currentProgram);

		// release shaders and files
		egpReleaseShader(shaders + 0);
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 0);
		egpReleaseFileContents(files + 1);
	}

	// test texture program
	// example of shared shader
	{
		// load shared shader
		files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs/drawTexture_fs4x.glsl");
		shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

		// pass texcoord
		{
			files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs/passTexcoord_vs4x.glsl");
			shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);

			// same as previous
			currentProgramIndex = testTextureProgramIndex;
			currentProgram = glslPrograms + currentProgramIndex;

			*currentProgram = egpCreateProgram();
			egpAttachShaderToProgram(currentProgram, shaders + 0);
			egpAttachShaderToProgram(currentProgram, shaders + 1);
			egpLinkProgram(currentProgram);
			egpValidateProgram(currentProgram);

			// release unique shaders and files
			egpReleaseShader(shaders + 0);
			egpReleaseFileContents(files + 0);
		}

		// pass texcoord, passthru position
		{
			files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs/passTexcoord_passthruPosition_vs4x.glsl");
			shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);

			currentProgramIndex = testTexturePassthruProgramIndex;
			currentProgram = glslPrograms + currentProgramIndex;

			*currentProgram = egpCreateProgram();
			egpAttachShaderToProgram(currentProgram, shaders + 0);
			egpAttachShaderToProgram(currentProgram, shaders + 1);
			egpLinkProgram(currentProgram);
			egpValidateProgram(currentProgram);

			// bloom should use this vertex shader, so let's keep nesting!
			{
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_bloom/brightpass_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = bloomBrightProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_bloom/blur_gaussian_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = bloomBlurProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_bloom/blend_screen_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = bloomBlendProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
			}
			// some deferred parts should use this vertex shader!
			{
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_deferred/deferredShading_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = deferredShadingProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_deferred/deferredComposite_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = deferredCompositeProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
				{
					files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs_deferred/depthofFieldComposite_fs4x.glsl");
					shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

					currentProgramIndex = depthOfFieldCompositeProgramIndex;
					currentProgram = glslPrograms + currentProgramIndex;

					*currentProgram = egpCreateProgram();
					egpAttachShaderToProgram(currentProgram, shaders + 0);
					egpAttachShaderToProgram(currentProgram, shaders + 2);
					egpLinkProgram(currentProgram);
					egpValidateProgram(currentProgram);

					egpReleaseShader(shaders + 2);
					egpReleaseFileContents(files + 2);
				}
			}

			// done with passthru vs
			egpReleaseShader(shaders + 0);
			egpReleaseFileContents(files + 0);
		}

		// release shared
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 1);
	}

	{ //load the line stuff because I don't care about sharing things like above.
		files[2] = egpLoadFileContents("../../../../resource/glsl/4x/fs/drawColor_fs4x.glsl");
		shaders[2] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[2].contents);

		files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs/transform_vs4x.glsl");
		shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);

		// draw curve using geometry shader
		{
			files[1] = egpLoadFileContents("../../../../resource/glsl/4x/gs/drawCurve_gs4x.glsl");
			shaders[1] = egpCreateShaderFromSource(EGP_SHADER_GEOMETRY, files[1].contents);

			currentProgramIndex = drawCurveProgram;
			currentProgram = glslPrograms + currentProgramIndex;
			*currentProgram = egpCreateProgram();
			egpAttachShaderToProgram(currentProgram, shaders + 0);
			egpAttachShaderToProgram(currentProgram, shaders + 1);
			egpAttachShaderToProgram(currentProgram, shaders + 2);
			egpLinkProgram(currentProgram);
			egpValidateProgram(currentProgram);

			egpReleaseShader(shaders + 1);
			egpReleaseFileContents(files + 1);
		}
		{
			files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs/drawSolid_fs4x.glsl");
			shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

			currentProgramIndex = testSolidColorProgramIndex;
			currentProgram = glslPrograms + currentProgramIndex;
			*currentProgram = egpCreateProgram();
			egpAttachShaderToProgram(currentProgram, shaders + 0);
			egpAttachShaderToProgram(currentProgram, shaders + 1);
			egpLinkProgram(currentProgram);
			egpValidateProgram(currentProgram);

			egpReleaseShader(shaders + 1);
			egpReleaseFileContents(files + 1);
		}

		egpReleaseShader(shaders + 0);
		egpReleaseFileContents(files + 0);
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 1);
		egpReleaseShader(shaders + 2);
		egpReleaseFileContents(files + 2);
	}

	// lighting
	{
		currentProgramIndex = phongProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;

		files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs/phong_vs4x.glsl");
		files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs/phong_fs4x.glsl");
		shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);
		shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

		*currentProgram = egpCreateProgram();
		egpAttachShaderToProgram(currentProgram, shaders + 0);
		egpAttachShaderToProgram(currentProgram, shaders + 1);
		egpLinkProgram(currentProgram);
		egpValidateProgram(currentProgram);

		egpReleaseShader(shaders + 0);
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 0);
		egpReleaseFileContents(files + 1);
	}

	// deferred rendering
	{
		currentProgramIndex = gbufferProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;

		files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs_deferred/passAttribs_world_vs4x.glsl");
		files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs_deferred/drawGBuffers_fs4x.glsl");
		shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);
		shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

		*currentProgram = egpCreateProgram();
		egpAttachShaderToProgram(currentProgram, shaders + 0);
		egpAttachShaderToProgram(currentProgram, shaders + 1);
		egpLinkProgram(currentProgram);
		egpValidateProgram(currentProgram);

		egpReleaseShader(shaders + 0);
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 0);
		egpReleaseFileContents(files + 1);
	}
	{
		currentProgramIndex = deferredLightPassProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;

		files[0] = egpLoadFileContents("../../../../resource/glsl/4x/vs_deferred/passPosition_clip_vs4x.glsl");
		files[1] = egpLoadFileContents("../../../../resource/glsl/4x/fs_deferred/phong_deferred_pointLight_fs4x.glsl");
		shaders[0] = egpCreateShaderFromSource(EGP_SHADER_VERTEX, files[0].contents);
		shaders[1] = egpCreateShaderFromSource(EGP_SHADER_FRAGMENT, files[1].contents);

		*currentProgram = egpCreateProgram();
		egpAttachShaderToProgram(currentProgram, shaders + 0);
		egpAttachShaderToProgram(currentProgram, shaders + 1);
		egpLinkProgram(currentProgram);
		egpValidateProgram(currentProgram);

		egpReleaseShader(shaders + 0);
		egpReleaseShader(shaders + 1);
		egpReleaseFileContents(files + 0);
		egpReleaseFileContents(files + 1);
	}


	// configure all uniforms at once
	for (currentProgramIndex = 0; currentProgramIndex < GLSLProgramCount; ++currentProgramIndex)
	{
		// get location of every uniform
		currentProgram = glslPrograms + currentProgramIndex;
		currentUniformSet = glslCommonUniforms[currentProgramIndex];
		egpActivateProgram(currentProgram);
		for (u = 0; u < GLSLCommonUniformCount; ++u)
			currentUniformSet[u] = egpGetUniformLocation(currentProgram, commonUniformName[u]);

		// bind constant uniform locations, if they exist, because they never change
		// e.g. image bindings
		egpSendUniformInt(currentUniformSet[unif_dm], UNIF_INT, 1, imageLocations);
		egpSendUniformInt(currentUniformSet[unif_sm], UNIF_INT, 1, imageLocations + 1);

		egpSendUniformInt(currentUniformSet[unif_img], UNIF_INT, 1, imageLocations);
		egpSendUniformInt(currentUniformSet[unif_img1], UNIF_INT, 1, imageLocations + 1);
		egpSendUniformInt(currentUniformSet[unif_img2], UNIF_INT, 1, imageLocations + 2);
		egpSendUniformInt(currentUniformSet[unif_img3], UNIF_INT, 1, imageLocations + 3);
		egpSendUniformInt(currentUniformSet[unif_img4], UNIF_INT, 1, imageLocations + 4);

		egpSendUniformInt(currentUniformSet[unif_img_light_diffuse], UNIF_INT, 1, imageLocations + 2);
		egpSendUniformInt(currentUniformSet[unif_img_light_specular], UNIF_INT, 1, imageLocations + 3);

		egpSendUniformInt(currentUniformSet[unif_img_position], UNIF_INT, 1, imageLocations + 4);
		egpSendUniformInt(currentUniformSet[unif_img_normal], UNIF_INT, 1, imageLocations + 5);
		egpSendUniformInt(currentUniformSet[unif_img_texcoord], UNIF_INT, 1, imageLocations + 6);
		egpSendUniformInt(currentUniformSet[unif_img_depth], UNIF_INT, 1, imageLocations + 7);
	}


	// disable all
	egpActivateProgram(0);
	egpActivateVAO(0);
}

void deleteShaders()
{
	// convenient way to release all programs
	unsigned int i;
	for (i = 0; i < GLSLProgramCount; ++i)
		egpReleaseProgram(glslPrograms + i);
}


// setup and delete framebuffers
void setupFramebuffers(unsigned int frameWidth, unsigned int frameHeight)
{
	// prepare framebuffers in one simple call

	{ //BLOOM
		const unsigned int frameWidth_d2 = frameWidth / 2;
		const unsigned int frameWidth_d4 = frameWidth / 4;
		const unsigned int frameWidth_d8 = frameWidth / 8;
		const unsigned int frameHeight_d2 = frameHeight / 2;
		const unsigned int frameHeight_d4 = frameHeight / 4;
		const unsigned int frameHeight_d8 = frameHeight / 8;
		unsigned int i;

		// one for the scene
		fbo[sceneFBO] = egpfwCreateFBO(frameWidth, frameHeight, 1, COLOR_RGBA16, DEPTH_D32, SMOOTH_NOWRAP);

		// bright pass
		fbo[brightFBO_d2] = egpfwCreateFBO(frameWidth_d2, frameHeight_d2, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);

		// blur passes
		fbo[hblurFBO_d2] = egpfwCreateFBO(frameWidth_d2, frameHeight_d2, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);
		fbo[vblurFBO_d2] = egpfwCreateFBO(frameWidth_d2, frameHeight_d2, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);
		fbo[hblurFBO_d4] = egpfwCreateFBO(frameWidth_d4, frameHeight_d4, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);
		fbo[vblurFBO_d4] = egpfwCreateFBO(frameWidth_d4, frameHeight_d4, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);
		fbo[hblurFBO_d8] = egpfwCreateFBO(frameWidth_d8, frameHeight_d8, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);
		fbo[vblurFBO_d8] = egpfwCreateFBO(frameWidth_d8, frameHeight_d8, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);

		// composite pass
		fbo[compositeFBO] = egpfwCreateFBO(frameWidth, frameHeight, 1, COLOR_RGBA16, DEPTH_DISABLE, SMOOTH_NOWRAP);


		// get inverted frame sizes
		// this represents the size of one pixel within SCREEN SPACE
		// since screen space is within [0, 1], one pixel = 1/size
		for (i = 0; i < fboCount; ++i)
			pixelSizeInv[i].set(
				1.0f / (float)(fbo + i)->frameWidth,
				1.0f / (float)(fbo + i)->frameHeight
			);
	}

	{ //DEFERRED
		const egpColorFormat colorFormat = COLOR_RGBA32F;

		// one for the scene geometry (MRT, one for each attrib)
		fbo[gbufferSceneFBO] = egpfwCreateFBO(frameWidth, frameHeight, 3, colorFormat, DEPTH_D32, SMOOTH_NOWRAP);

		// deferred shading
		fbo[deferredShadingFBO] = egpfwCreateFBO(frameWidth, frameHeight, 1, colorFormat, DEPTH_DISABLE, SMOOTH_NOWRAP);

		// light pre-pass (MRT, one for diffuse, one for specular lighting)
		fbo[lightPassFBO] = egpfwCreateFBO(frameWidth, frameHeight, 2, colorFormat, DEPTH_DISABLE, SMOOTH_NOWRAP);

		// deferred lighting composite pass
		fbo[deferredLightingCompositeFBO] = egpfwCreateFBO(frameWidth, frameHeight, 1, colorFormat, DEPTH_DISABLE, SMOOTH_NOWRAP); 
	}

	{ //Depth of Field
		const egpColorFormat colorFormat = COLOR_RGBA32F;

		fbo[depthOfFieldOutputFBO] = egpfwCreateFBO(frameWidth, frameHeight, 2, colorFormat, DEPTH_DISABLE, SMOOTH_NOWRAP);
	}

	{ //Curves
		const egpColorFormat colorFormat = COLOR_RGBA32F;
		fbo[curvesFBO] = egpfwCreateFBO(frameWidth, frameHeight, 1, COLOR_RGB16, DEPTH_DISABLE, SMOOTH_NOWRAP);
	}
}

void deleteFramebuffers()
{
	// how convenient
	for (unsigned int i = 0; i < fboCount; ++i)
		egpfwReleaseFBO(fbo + i);
}

// setup the different render paths
void setupScenePathBloom()
{
	//Create a pass to render the moon and earth (skybox is handled somewhere else).
	RenderPass moonPass(fbo, glslPrograms), earthPass(fbo, glslPrograms);

	//Copy the same settings from before for the rendering.
	moonPass.setProgram(testTextureProgramIndex);
	moonPass.setPipelineStage(sceneFBO);
	moonPass.setVAO(vao + sphere8x6Model);
	moonPass.addTexture(RenderPassTextureData(GL_TEXTURE_2D, GL_TEXTURE0, tex[moonTexHandle_dm]));
	moonPass.addUniform(render_pass_uniform_float_matrix(glslCommonUniforms[testTextureProgramIndex][unif_mvp], 1, 0, &moonModelViewProjectionMatrix));

	earthPass.setProgram(phongProgramIndex);
	earthPass.setPipelineStage(sceneFBO);
	earthPass.setVAO(vao + sphereHiResObjModel);
	earthPass.addTexture(RenderPassTextureData(GL_TEXTURE_2D, GL_TEXTURE1, tex[earthTexHandle_sm]));
	earthPass.addTexture(RenderPassTextureData(GL_TEXTURE_2D, GL_TEXTURE0, tex[earthTexHandle_dm]));
	earthPass.addUniform(render_pass_uniform_float(glslCommonUniforms[phongProgramIndex][unif_eyePos], UNIF_VEC4, 1, eyePos_object.v));
	earthPass.addUniform(render_pass_uniform_float(glslCommonUniforms[phongProgramIndex][unif_lightPos], UNIF_VEC4, 1, lightPos_object.v));
	earthPass.addUniform(render_pass_uniform_float_matrix(glslCommonUniforms[phongProgramIndex][unif_mvp], 1, 0, &earthModelViewProjectionMatrix));

	//Add them to the global render path.
	globalRenderPath.addRenderPass(moonPass);
	globalRenderPath.addRenderPass(earthPass);
}

void setupEffectPathBloom()
{
	//Create passes for all the individual steps for bloom.
	RenderPass brightPass(fbo, glslPrograms),
		hblur1(fbo, glslPrograms), vblur1(fbo, glslPrograms),
		hblur2(fbo, glslPrograms), vblur2(fbo, glslPrograms),
		hblur3(fbo, glslPrograms), vblur3(fbo, glslPrograms), 
		composite(fbo, glslPrograms);

	//If you look at it too long, this all blurs together (ba dum tss) but basically it
	//just sets up the different steps for bloom. Nothing here is different
	//than from how it was when it was in the renderScene function.

	//NOTE: The blurs use render_pass_uniform_float_complex because they are referencing variables
	//that don't actually exist; they need a VEC2 with one component in an actual stored vector, and 
	//the other component set to 0. We give it a "source" of sorts that the RenderPass will use
	//later on to create an actual VEC2 to be passed as a uniform.
	//	...a simpler solution would've been to just make pixelSizeInvHorizontal and pixelSizeInvVertical variables...

	//Bright pass
	brightPass.setProgram(bloomBrightProgramIndex);
	brightPass.setVAO(vao + fsqModel);
	brightPass.setPipelineStage(brightFBO_d2);
	brightPass.addColorTarget(FBOTargetColorTexture(sceneFBO, 0, 0));

	currentUniformSet = glslCommonUniforms[bloomBlurProgramIndex];

	//First horizontal blur
	hblur1.setProgram(bloomBlurProgramIndex);
	hblur1.setPipelineStage(hblurFBO_d2);
	hblur1.addColorTarget(FBOTargetColorTexture(brightFBO_d2, 0, 0));
	hblur1.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[brightFBO_d2].x, &CONST_ZERO_FLOAT }));

	//First vertical blur
	vblur1.setProgram(bloomBlurProgramIndex);
	vblur1.setPipelineStage(vblurFBO_d2);
	vblur1.addColorTarget(FBOTargetColorTexture(hblurFBO_d2, 0, 0));
	vblur1.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d2].y }));

	//Second horizontal blur
	hblur2.setProgram(bloomBlurProgramIndex);
	hblur2.setPipelineStage(hblurFBO_d4);
	hblur2.addColorTarget(FBOTargetColorTexture(vblurFBO_d2, 0, 0));
	hblur2.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[vblurFBO_d2].x, &CONST_ZERO_FLOAT }));

	//Second vertical blur
	vblur2.setProgram(bloomBlurProgramIndex);
	vblur2.setPipelineStage(vblurFBO_d4);
	vblur2.addColorTarget(FBOTargetColorTexture(hblurFBO_d4, 0, 0));
	vblur2.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d4].y }));

	//Third horizontal blur
	hblur3.setProgram(bloomBlurProgramIndex);
	hblur3.setPipelineStage(hblurFBO_d8);
	hblur3.addColorTarget(FBOTargetColorTexture(vblurFBO_d4, 0, 0));
	hblur3.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[vblurFBO_d4].x, &CONST_ZERO_FLOAT }));

	//Third vertical blur
	vblur3.setProgram(bloomBlurProgramIndex);
	vblur3.setPipelineStage(vblurFBO_d8);
	vblur3.addColorTarget(FBOTargetColorTexture(hblurFBO_d8, 0, 0));
	vblur3.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d8].y }));

	//Finally, composite them all together with the last pass.
	composite.setProgram(bloomBlendProgramIndex);
	composite.setPipelineStage(compositeFBO);
	composite.addColorTarget(FBOTargetColorTexture(sceneFBO, 0, 0));
	composite.addColorTarget(FBOTargetColorTexture(vblurFBO_d2, 1, 0));
	composite.addColorTarget(FBOTargetColorTexture(vblurFBO_d4, 2, 0));
	composite.addColorTarget(FBOTargetColorTexture(vblurFBO_d8, 3, 0));
	
	//Add them all to the render path.
	globalRenderPath.addRenderPasses({ brightPass, hblur1, vblur1, hblur2, vblur2, hblur3, vblur3, composite });
}

void setupNetgraphPathBloom()
{
	globalRenderNetgraph.clearFBOList();

	//For the Bloom netgraph, we display the scene, bright, vertical blurs, and composite.
	globalRenderNetgraph.addFBOs({
		FBOTargetColorTexture(sceneFBO, 0, 0),
		FBOTargetColorTexture(brightFBO_d2, 0, 0),
		FBOTargetColorTexture(vblurFBO_d2, 0, 0),
		FBOTargetColorTexture(vblurFBO_d4, 0, 0),
		FBOTargetColorTexture(vblurFBO_d8, 0, 0),
		FBOTargetColorTexture(compositeFBO, 0, 0),
	});
}

void setupScenePathDeferred()
{
	//Create passes to render the individual objects in the scene.
	currentUniformSet = glslCommonUniforms[gbufferProgramIndex];
	RenderPass earthPass(fbo, glslPrograms), moonPass(fbo, glslPrograms), marsPass(fbo, glslPrograms), groundPass(fbo, glslPrograms);

	earthPass.setProgram(gbufferProgramIndex);
	earthPass.setPipelineStage(gbufferSceneFBO);
	earthPass.setVAO(vao + sphereHiResObjModel);
	earthPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_viewprojMat], 1, 0, &viewProjMat));
	earthPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_modelMat], 1, 0, &earthModelMatrix));
	earthPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_atlasMat], 1, 0, &earthAtlasMatrix));

	moonPass.setProgram(gbufferProgramIndex);
	moonPass.setPipelineStage(gbufferSceneFBO);
	moonPass.setVAO(vao + sphereLowResObjModel);
	moonPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_viewprojMat], 1, 0, &viewProjMat));
	moonPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_modelMat], 1, 0, &moonModelMatrix));
	moonPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_atlasMat], 1, 0, &moonAtlasMatrix));

	marsPass.setProgram(gbufferProgramIndex);
	marsPass.setPipelineStage(gbufferSceneFBO);
	marsPass.setVAO(vao + sphereLowResObjModel);
	marsPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_viewprojMat], 1, 0, &viewProjMat));
	marsPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_modelMat], 1, 0, &marsModelMatrix));
	marsPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_atlasMat], 1, 0, &marsAtlasMatrix));

	groundPass.setProgram(gbufferProgramIndex);
	groundPass.setPipelineStage(gbufferSceneFBO);
	groundPass.setVAO(vao + fsqModel);
	groundPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_viewprojMat], 1, 0, &viewProjMat));
	groundPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_modelMat], 1, 0, &groundModelMatrix));
	groundPass.addUniform(render_pass_uniform_float_matrix(currentUniformSet[unif_atlasMat], 1, 0, &groundAtlasMatrix));

	//Add them to the actual render path.
	globalRenderPath.addRenderPasses({ earthPass, moonPass, marsPass, groundPass });
}

void setupEffectPathDeferred()
{
	//Create a pass for the actual deferred shading.
	RenderPass deferredPass(fbo, glslPrograms);

	deferredPass.setProgram(deferredShadingProgramIndex);
	deferredPass.setPipelineStage(deferredShadingFBO);
	deferredPass.setVAO(vao + fsqModel);

	currentUniformSet = glslCommonUniforms[deferredShadingProgramIndex];

	//Add all the uniforms and textures
	deferredPass.addDepthTarget(FBOTargetDepthTexture(gbufferSceneFBO, 7));
	deferredPass.addColorTarget(FBOTargetColorTexture(gbufferSceneFBO, 6, 2));
	deferredPass.addColorTarget(FBOTargetColorTexture(gbufferSceneFBO, 5, 1));
	deferredPass.addColorTarget(FBOTargetColorTexture(gbufferSceneFBO, 4, 0));

	deferredPass.addTexture(RenderPassTextureData(GL_TEXTURE_2D, GL_TEXTURE1, tex[atlas_specular]));
	deferredPass.addTexture(RenderPassTextureData(GL_TEXTURE_2D, GL_TEXTURE0, tex[atlas_diffuse]));
	deferredPass.addUniform(render_pass_uniform_float(currentUniformSet[unif_eyePos], UNIF_VEC4, 1, cameraPosWorld.v));

	deferredPass.addUniform(render_pass_uniform_float(currentUniformSet[unif_lightColor], UNIF_VEC4, numLightsShading, lightColor->v));
	deferredPass.addUniform(render_pass_uniform_float(currentUniformSet[unif_lightPos], UNIF_VEC4, numLightsShading, lightPos_world->v));

	//Add it to the actual render path.
	globalRenderPath.addRenderPass(deferredPass);
}

void setupNetgraphPathDeferred()
{
	globalRenderNetgraph.clearFBOList();

	//For the Deferred netgraph, we display the different channels in the scene gbuffer and the final result
	globalRenderNetgraph.addFBOs({
		FBOTargetColorTexture(gbufferSceneFBO, 0, 0),
		FBOTargetColorTexture(gbufferSceneFBO, 0, 1),
		FBOTargetColorTexture(gbufferSceneFBO, 0, 2),
		FBOTargetColorTexture(deferredShadingFBO, 0, 0),
	});
}

void setupEffectPathDOF()
{
	RenderPass
		hblur1(fbo, glslPrograms), vblur1(fbo, glslPrograms),
		hblur2(fbo, glslPrograms), vblur2(fbo, glslPrograms),
		hblur3(fbo, glslPrograms), vblur3(fbo, glslPrograms),
		dofComposite(fbo, glslPrograms);

	currentUniformSet = glslCommonUniforms[bloomBlurProgramIndex];

	//First horizontal blur
	hblur1.setProgram(bloomBlurProgramIndex);
	hblur1.setVAO(vao + fsqModel);
	hblur1.setPipelineStage(hblurFBO_d2);
	hblur1.addColorTarget(FBOTargetColorTexture(sceneFBO, 0, 0));
	hblur1.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[brightFBO_d2].x, &CONST_ZERO_FLOAT }));

	//First vertical blur
	vblur1.setPipelineStage(vblurFBO_d2);
	vblur1.addColorTarget(FBOTargetColorTexture(hblurFBO_d2, 0, 0));
	vblur1.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d2].y }));

	//Second horizontal blur
	hblur2.setPipelineStage(hblurFBO_d4);
	hblur2.addColorTarget(FBOTargetColorTexture(vblurFBO_d2, 0, 0));
	hblur2.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[vblurFBO_d2].x, &CONST_ZERO_FLOAT }));

	//Second vertical blur
	vblur2.setPipelineStage(vblurFBO_d4);
	vblur2.addColorTarget(FBOTargetColorTexture(hblurFBO_d4, 0, 0));
	vblur2.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d4].y }));

	//Third horizontal blur
	hblur3.setPipelineStage(hblurFBO_d8);
	hblur3.addColorTarget(FBOTargetColorTexture(vblurFBO_d4, 0, 0));
	hblur3.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &pixelSizeInv[vblurFBO_d4].x, &CONST_ZERO_FLOAT }));

	//Third vertical blur
	vblur3.setPipelineStage(vblurFBO_d8);
	vblur3.addColorTarget(FBOTargetColorTexture(hblurFBO_d8, 0, 0));
	vblur3.addUniform(render_pass_uniform_float_complex(currentUniformSet[unif_pixelSizeInv], UNIF_VEC2, 1, { &CONST_ZERO_FLOAT, &pixelSizeInv[hblurFBO_d8].y }));

	currentUniformSet = glslCommonUniforms[depthOfFieldCompositeProgramIndex];

	dofComposite.setPipelineStage(depthOfFieldOutputFBO);
	dofComposite.setProgram(depthOfFieldCompositeProgramIndex);

	dofComposite.addDepthTarget(FBOTargetDepthTexture(sceneFBO, 0));
	dofComposite.addColorTarget(FBOTargetColorTexture(sceneFBO, 1, 0));
	dofComposite.addColorTarget(FBOTargetColorTexture(vblurFBO_d2, 2, 0));
	dofComposite.addColorTarget(FBOTargetColorTexture(vblurFBO_d4, 3, 0));
	dofComposite.addColorTarget(FBOTargetColorTexture(vblurFBO_d8, 4, 0));

	globalRenderPath.addRenderPasses({ hblur1, vblur1, hblur2, vblur2, hblur3, vblur3, dofComposite });
}

void setupNetgraphPathDOF()
{
	globalRenderNetgraph.clearFBOList();
	globalRenderNetgraph.addFBOs({
		FBOTargetColorTexture(sceneFBO, 0, 0),
		FBOTargetColorTexture(vblurFBO_d2, 0, 0),
		FBOTargetColorTexture(vblurFBO_d4, 0, 0),
		FBOTargetColorTexture(vblurFBO_d8, 0, 0),
		FBOTargetColorTexture(depthOfFieldOutputFBO, 0, 1),
		FBOTargetColorTexture(depthOfFieldOutputFBO, 0, 0),
	});
}

void setupRenderPaths()
{
	globalRenderPath.clearAllPasses();

	switch (currentRenderMode)
	{
		case bloomRenderMethod:
			setupScenePathBloom();
			setupEffectPathBloom();
			setupNetgraphPathBloom();
			displayMode = deferredShadingFBO;
			break;
		case deferredRenderMethod:
			setupScenePathDeferred();
			setupEffectPathDeferred();
			setupNetgraphPathDeferred();
			displayMode = compositeFBO;
			break;
		case depthOfFieldRenderMethod:
			setupScenePathBloom();
			setupEffectPathDOF();
			setupNetgraphPathDOF();
			displayMode = compositeFBO;
			break;
		case numRenderMethods:
		default: 
			break;
	}

}


// restart all timers and time counters
void resetTimers()
{
	const double renderFPS = 30.0;
	const double secondFPS = 1.0;

	egpTimerSet(renderTimer, renderFPS);
	egpTimerStart(renderTimer);

	egpTimerSet(secondTimer, secondFPS);
	egpTimerStart(secondTimer);
}


// configure viewport for correctly drawing to the back buffer
void drawToBackBuffer(int x, int y, unsigned int w, unsigned int h)
{
	// tell OpenGL to draw directly to back buffer
	glDrawBuffer(GL_BACK);

	// reset buffer tests
	// assume only FBOs are using depth
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_STENCIL_TEST);

	// reset viewport with borders clipped
	glViewport(x, y, w, h);
}


//-----------------------------------------------------------------------------

// update camera
void updateCameraControlled(float dt, egpMouse *mouse, bool allowInput)
{
	// TRACK MOUSE AND KEYS TO CONTROL CAMERA: 
	// remember, the view transform is the INVERSE of the camera's model transform, 
	//	so we can start by making a proper camera transform then invert it
	// 1. compute deltas (velocities)
	// 2. integrate!
	// 3. update view matrix (camera's transformation)
	if (allowInput && egpMouseIsButtonDown(mouse, 0))
	{
		cameraElevation -= (float)egpMouseDeltaY(mouse) * dt * cameraRotateSpeed;
		cameraAzimuth -= (float)egpMouseDeltaX(mouse) * dt * cameraRotateSpeed;
	}

	viewMatrix = cbtk::cbmath::makeRotationEuler4ZYX(cameraElevation, cameraAzimuth, 0.0f);

	// apply current rotation to our movement vector so that "forward" is the direction the camera is facing
	deltaCamPos.set(
		(float)egpKeyboardDifference(keybd, 'd', 'a'),
		(float)egpKeyboardDifference(keybd, 'e', 'q'),
		(float)egpKeyboardDifference(keybd, 's', 'w'),
		0.0f);
	deltaCamPos = viewMatrix * deltaCamPos;

	cameraPosWorld += cbtk::cbmath::normalize(deltaCamPos) * dt * cameraMoveSpeed;
}

void updateCameraOrbit(float dt)
{
	// force camera to orbit around center of world
	cameraAzimuth += dt * cameraRotateSpeed;

	viewMatrix = cbtk::cbmath::makeRotationEuler4ZYX(0.0f, cameraAzimuth, 0.0f);

	cameraPosWorld.set(sinf(cameraAzimuth)*cameraDistance, 0.0f, cosf(cameraAzimuth)*cameraDistance, 1.0f);
}


//-----------------------------------------------------------------------------

// initialize game objects
// mainly for good memory management, handling ram and vram
int initGame()
{
	// setup framebuffers
	// don't bother with this here actually... see window resize callback
	//	setupFramebuffers();

	// setup geometry
	setupGeometry();

	// setup textures
	setupTextures();

	// setup shaders
	setupShaders();

	// setup paths and passes
	setupRenderPaths();

	// other
	egpSetActiveMouse(mouse);
	egpSetActiveKeyboard(keybd);

	// done
	return 1;
}

// destroy game objects (mem)
int termGame()
{
	// good practice to do this in reverse order of creation
	//	in case something is referencing something else

	// delete fbos
	deleteFramebuffers();

	// delete shaders
	deleteShaders();

	// delete textures
	deleteTextures();

	// delete geometry
	deleteGeometry();

	// done
	return 1;
}


// output controls
void displayControls()
{
	printf("\n-------------------------------------------------------");
	printf("\n CONTROLS: \n");
	printf("\n wasdqe = camera movement");
	printf("\n ~` = display controls");
	printf("\n o = toggle slow-mo playback for all");
	printf("\n p = toggle play/pause for all");

	printf("\n l = real-time reload all shaders");
	printf("\n x = toggle coordinate axes post-draw");

	//printf("\n 0-9 = toggle pipeline stage to be displayed");
	printf("\n 1-6 = change the keyframe control channel");
	printf("\n 7 = switch to keyframe scrub mode");
	printf("\n space = toggle whether the keyframe window is paused");
	printf("\n y = clear all keyframes");

	printf("\n-------------------------------------------------------");
}


// process input each frame
void handleInputState(float dt)
{
	// display controls
	if (egpKeyboardIsKeyPressed(keybd, '`') ||
		egpKeyboardIsKeyPressed(keybd, '~'))
		displayControls();

	// slowmo: play back at 25%
	if (egpKeyboardIsKeyPressed(keybd, 'o'))
		playrate = (playrate != 100) ? 100 : 25;

	// pause/play
	if (egpKeyboardIsKeyPressed(keybd, 'p'))
		playing = 1 - playing;


	// reload shaders
	if (egpKeyboardIsKeyPressed(keybd, 'l'))
	{
		deleteShaders();
		setupShaders();
	}

	// toggle axes
	if (egpKeyboardIsKeyPressed(keybd, 'x'))
		testDrawAxes = 1 - testDrawAxes;

	if (egpKeyboardIsKeyPressed(keybd, 'm'))
	{
		++currentRenderMode;
		setupRenderPaths();
	}

	if (egpKeyboardIsKeyPressed(keybd, 'n'))
		displayNetgraphToggle = !displayNetgraphToggle;

	// toggle pipeline stage
	if (egpKeyboardIsKeyPressed(keybd, '0'))
	{
		displayMode = fboCount;
		fboFinalDisplay = fbo;
	}
	else
	{
		/*
		for (unsigned char c = '1', i = 0; c <= '9'; ++c, ++i)
			if (egpKeyboardIsKeyPressed(keybd, c))
			{
				displayMode = i;
			}
		*/
	}


	// place waypoints
	bool mouseInKeyframeWindow = keyframeWindow.updateInput(mouse, keybd);
	
	updateCameraControlled(dt, mouse, !mouseInKeyframeWindow);

	// finish by updating input state
	egpMouseUpdate(mouse);
	egpKeyboardUpdate(keybd);
}

// game update to perform each frame
// update game state using the time since the last update (dt) in seconds
void updateGameState(float dt)
{
	// update camera
//	updateCameraOrbit(dt);

	// update view matrix
	// 'c3' in a 4x4 matrix is the translation part
	viewMatrix.c3 = cameraPosWorld;
	viewMatrix = cbtk::cbmath::transformInverseNoScale(viewMatrix);
	viewProjMat = projectionMatrix * viewMatrix;


	// update game objects
	// scale time first
	dt *= (float)(playrate * playing) * 0.01f;

	keyframeWindow.update(dt);

	// update objects here

	// SKYBOX TRANSFORM: 
	{
		// easy way: exact same as the camera's transform (view), but 
		//	we remove the translation (which makes the background seem 
		//	infinitely far away), and scale up instead

		/*
		// multiply view matrix with scale matrix to make modelview
		skyboxModelViewMatrix = viewMatrix * cbtk::cbmath::makeScale4(minClipDist);

		// remove translation component (by setting 4th column to 0,0,0,1)
		skyboxModelViewMatrix.c3 = cbmath::v4w;

		// concatenate with proj to get mvp
		skyboxModelViewProjectionMatrix = projectionMatrix * skyboxModelViewMatrix;*/

		skyboxModelMatrix = cbtk::cbmath::makeScaleTranslate(minClipDist, cameraPosWorld.xyz);

		// concatenate with proj to get mvp
		skyboxModelViewProjectionMatrix = viewProjMat * skyboxModelMatrix;
	}

	// earth: 
	{
		// animate daytime and orbit
		earthDaytime += dt * earthDaytimePeriod;
		earthOrbit += dt * earthOrbitPeriod;

		// calculate model matrix
		//earthModelMatrix = cbmath::makeRotationZ4(earthTilt) * cbmath::makeRotationY4(earthDaytime);
		earthModelMatrix = cbmath::makeRotationEuler4XYZ(keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_ROT_X) * 3.14f * 2.0f,
			keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_ROT_Y) * 3.14f * 2.0f,
			keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_ROT_Z) * 3.14f * 2.0f);

		earthModelMatrix.c3.x = keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_POS_X) * 5.0f;
		earthModelMatrix.c3.y = keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_POS_Y) * 5.0f;
		earthModelMatrix.c3.z = keyframeWindow.getValAtCurrentTime(KeyframeWindow::CHANNEL_POS_Z) * 5.0f;

		// update mvp
		earthModelViewProjectionMatrix = viewProjMat * earthModelMatrix;

		// update inverse
		earthModelInverseMatrix = cbmath::transformInverseNoScale(earthModelMatrix);

		// variables used for the bloom path
		eyePos_object = earthModelInverseMatrix * cameraPosWorld;
		lightPos_object = earthModelInverseMatrix * lightPos_world[3];
	}

	// moon: 
	{
		moonOrbit += dt * moonOrbitPeriod;

		moonModelMatrix = cbmath::makeRotationZ4(moonTilt) * cbmath::makeRotationY4(moonOrbit) * cbmath::makeScale4(moonSize);
		moonModelMatrix.c3.x = earthModelMatrix.c3.x + cosf(moonOrbit) * moonDistance;
		moonModelMatrix.c3.y = earthModelMatrix.c3.y;
		moonModelMatrix.c3.z = earthModelMatrix.c3.z - sinf(moonOrbit) * moonDistance;

		moonModelViewProjectionMatrix = viewProjMat * moonModelMatrix;
		moonModelInverseMatrix = cbmath::transformInverseNoScale(moonModelMatrix);
	}

	// mars: 
	{
		marsModelMatrix = moonModelMatrix;
		marsModelMatrix.c3.y += 2.0f;
	}
}


// skybox clear
void renderSkybox()
{
	if (currentRenderMode == bloomRenderMethod || currentRenderMode == depthOfFieldRenderMethod)
	{
		egpfwActivateFBO(fbo + sceneFBO);
		currentProgramIndex = testTextureProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;
		currentUniformSet = glslCommonUniforms[currentProgramIndex];
		egpActivateProgram(currentProgram);

		// draw skybox instead of clearing
		glCullFace(GL_FRONT);
		glDepthFunc(GL_ALWAYS);
		glBindTexture(GL_TEXTURE_2D, tex[skyboxTexHandle]);

		egpSendUniformFloatMatrix(currentUniformSet[unif_mvp], UNIF_MAT4, 1, 0, skyboxModelViewProjectionMatrix.m);
		egpActivateVAO(vao + skyboxModel);
		egpDrawActiveVAO();

		glDepthFunc(GL_LESS);
		glCullFace(GL_BACK);
	}
	else
	{
		/**/
		egpfwActivateFBO(fbo + gbufferSceneFBO);
		currentProgramIndex = gbufferProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;
		currentUniformSet = glslCommonUniforms[currentProgramIndex];
		egpActivateProgram(currentProgram);

		// send common uniforms first
		egpSendUniformFloatMatrix(currentUniformSet[unif_viewprojMat], UNIF_MAT4, 1, 0, viewProjMat.m);

		// background
		{
			// normal scaling adjustment values
			const float normalScale[2] = { -1.0f, +1.0f };

			glCullFace(GL_FRONT);
			glDepthFunc(GL_ALWAYS);
			egpSendUniformFloat(currentUniformSet[unif_normalScale], UNIF_FLOAT, 1, normalScale);	// invert

			egpSendUniformFloatMatrix(currentUniformSet[unif_modelMat], UNIF_MAT4, 1, 0, skyboxModelMatrix.m);
			egpSendUniformFloatMatrix(currentUniformSet[unif_atlasMat], UNIF_MAT4, 1, 0, skyboxAtlasMatrix.m);
			egpActivateVAO(vao + skyboxModel);
			egpDrawActiveVAO();

			egpSendUniformFloat(currentUniformSet[unif_normalScale], UNIF_FLOAT, 1, normalScale + 1);
			glDepthFunc(GL_LESS);
			glCullFace(GL_BACK);
		}
	}
}


// draw curves
void renderCurve()
{
	const cbmath::vec4 waypointColor(1.0, 0.5f, 0.0f, 1.0f);
	const cbmath::vec4 objectColor(1.0f, 1.0f, 0.5f, 1.0f);

	int i;
	cbmath::vec4 *waypointPtr;
	cbmath::mat4 waypointMVP;

	// clear
	glClear(GL_COLOR_BUFFER_BIT);


	// draw curve
	currentProgramIndex = drawCurveProgram;
	currentProgram = glslPrograms + currentProgramIndex;
	currentUniformSet = glslCommonUniforms[currentProgramIndex];
	egpActivateProgram(currentProgram);
	egpSendUniformFloatMatrix(currentUniformSet[unif_mvp], UNIF_MAT4, 1, 0, curveDrawingProjectionMatrix.m);

	// ship waypoint data to program, where it will be received by GS
	egpSendUniformFloat(currentUniformSet[unif_waypoint], UNIF_VEC4, waypointCount, waypoint->v);
	egpSendUniformInt(currentUniformSet[unif_waypointCount], UNIF_INT, 1, &waypointCount);
	egpSendUniformInt(currentUniformSet[unif_curveMode], UNIF_INT, 1, &curveMode);
	egpSendUniformInt(currentUniformSet[unif_useWaypoints], UNIF_INT, 1, &useWaypoints);

	egpActivateVAO(vao + pointModel);
	egpDrawActiveVAO();

	// draw waypoints using solid color program and sphere model
	currentProgramIndex = testSolidColorProgramIndex;
	currentProgram = glslPrograms + currentProgramIndex;
	currentUniformSet = glslCommonUniforms[currentProgramIndex];
	egpActivateProgram(currentProgram);
	egpSendUniformFloat(currentUniformSet[unif_color], UNIF_VEC4, 1, waypointColor.v);

	egpActivateVAO(vao + sphere8x6Model);

	// draw waypoints
	for (i = 0, waypointPtr = waypoint; i < waypointCount; ++i, ++waypointPtr)
	{
		// set position, update MVP for this waypoint and draw
		waypointModelMatrix.c3 = *waypointPtr;
		waypointMVP = curveDrawingProjectionMatrix * waypointModelMatrix;
		egpSendUniformFloatMatrix(currentUniformSet[unif_mvp], UNIF_MAT4, 1, 0, waypointMVP.m);
		egpDrawActiveVAO();
	}


	// reset model matrix position
	waypointModelMatrix.c3 = cbmath::v4w;
}

// draw frame
// DRAWING AND UPDATING SHOULD BE SEPARATE (good practice)
void renderGameState()
{
	// first pass: scene

	renderSkybox(); //We "hardcode" this because it requires special GL calls that the RenderPass can't handle.

	// Complete our currently selected render path (draws all objects, and whatever other passes are needed).
	globalRenderPath.render();

	//egpfwActivateFBO(fbo + curvesFBO);
	//renderCurve();
	keyframeWindow.renderToFBO(glslCommonUniforms[drawCurveProgram], glslCommonUniforms[testSolidColorProgramIndex]);

	//-----------------------------------------------------------------------------
	//-----------------------------------------------------------------------------
	// FINAL DISPLAY: RENDER FINAL IMAGE ON FSQ TO BACK BUFFER

	// use back buffer
	egpfwActivateFBO(0);
	drawToBackBuffer(viewport_nb, viewport_nb, viewport_tw, viewport_th);

	// final pass: draw FSQ with user-selected FBO on it
	{
		// use texturing program, no mvp
		currentProgramIndex = testTexturePassthruProgramIndex;
		currentProgram = glslPrograms + currentProgramIndex;
		egpActivateProgram(currentProgram);
		egpActivateVAO(vao + fsqModel);

		// Get the fbo we want by grabbing it directly from the netgraph (whether it's visible or not).
		FBOTargetColorTexture bg = globalRenderNetgraph.getFBOAtIndex(displayMode);
		egpfwBindColorTargetTexture(fbo + bg.fboIndex, 0, bg.targetIndex);
		egpDrawActiveVAO();

		keyframeWindow.renderToBackbuffer(glslCommonUniforms[testTextureProgramIndex]);
		
		/*if (displayNetgraphToggle)
			globalRenderNetgraph.render();*/

		// done with textures
		glBindTexture(GL_TEXTURE_2D, 0);

		// TEST DRAW: coordinate axes at center of spaces
		//	and other line objects
		if (testDrawAxes)
		{
			currentProgramIndex = testColorProgramIndex;
			currentProgram = glslPrograms + currentProgramIndex;
			currentUniformSet = glslCommonUniforms[currentProgramIndex];
			egpActivateProgram(currentProgram);
			egpSendUniformFloatMatrix(currentUniformSet[unif_mvp], UNIF_MAT4, 1, 0, viewProjMat.m);

			// center of world
			// (this is useful to see where the origin is and how big one unit is)
			egpActivateVAO(vao + axesModel);
			egpDrawActiveVAO();
		}

		// done
		glEnable(GL_DEPTH_TEST);
	}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

	// disable all renderables, shaders
	egpActivateProgram(0);
	egpActivateVAO(0);
}



//-----------------------------------------------------------------------------
// callbacks

// process everything
int idle()
{
	int ret = 0;
	// timer checks
	// pro tip: see what happens if you don't do the 'if'  ;)

	// trigger render if it is time
	if (egpTimerUpdate(renderTimer))
	{
		///////////////////////////////////////////////////////////////////////
		updateGameState(renderTimer->dt);
		handleInputState(renderTimer->dt);
		renderGameState();
		///////////////////////////////////////////////////////////////////////
		ret = 1;
	}

//	if (egpTimerUpdate(secondTimer))
//	{
//		printf("\n %u frames rendered over %u seconds", renderTimer->ticks, secondTimer->ticks);
//	}

	return ret;
}

// window closed
void onCloseWindow()
{
	// clean up
	termGame();
}

// window resized
void onResizeWindow(int w, int h)
{
	// set new sizes
	win_w = w;
	win_h = h;
	win_aspect = ((float)w) / ((float)h);

	// calculate total viewport size
	viewport_tw = w + viewport_tb;
	viewport_th = h + viewport_tb;

	// update projection matrix
	projectionMatrix = cbtk::cbmath::makePerspective(fovy, win_aspect, znear, zfar);
	viewProjMat = projectionMatrix * viewMatrix;

	// update max clipping distance using pythagorean theorem 
	//	to far corner of frustum; the purpose of this is to 
	//	know how much space there is make the most of (skybox)
	{
		const float tf = zfar*tanf(fovy*0.5f);
		const float rf = tf*win_aspect;
		maxClipDist = (float)((unsigned int)(sqrtf(rf*rf + tf*tf + zfar*zfar)));
		minClipDist = (float)((unsigned int)(sqrtf(zfar*zfar / 3.0f)));
	}

	// if framebuffers are backed against the size of the main window, then 
	//	it's probably a good idea to tear down and remake the framebuffers...
	deleteFramebuffers();
	setupFramebuffers(viewport_tw, viewport_th);

	//-----------------------------------------------------------------------------
	// setup curve drawing camera
	// just an ortho projection matrix the same size as the window's viewport

	{
		const float tmpNF = 100.0f;
		curveDrawingProjectionMatrix = cbmath::m4Identity;
		curveDrawingProjectionMatrix.m00 = 2.0f / (float)viewport_tw;
		curveDrawingProjectionMatrix.m11 = 2.0f / (float)viewport_th;
		curveDrawingProjectionMatrix.m22 = -1.0f / tmpNF;
		curveDrawingProjectionMatrix.m30 = -(float)win_w / (float)viewport_tw;
		curveDrawingProjectionMatrix.m31 = -(float)win_h / (float)viewport_th;
		
		keyframeWindow.updateWindowSize(viewport_tw, viewport_th, tmpNF, win_w, win_h);
	}

}

// window moved
void onPositionWindow(int x, int y)
{
	win_x = x;
	win_y = y;
}
