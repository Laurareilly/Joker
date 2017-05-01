#pragma once

// GLSL program handle objects: 
// very convenient way to organize programs and their uniforms!
enum GLSLProgramIndex
{
	testColorProgramIndex,
	testTextureProgramIndex,
	testTexturePassthruProgramIndex,

	phongProgramIndex,
	silhouetteOutlineProgramIndex,
	celshadeProgramIndex,
	testTransformProgramIndex,

	// bloom
	bloomBrightProgramIndex,
	bloomBlurProgramIndex,
	bloomBlendProgramIndex,

	// shadow mapping and projective texturing
	projectiveTextureProgram,
	shadowMapProgram,


	// deferred rendering
	gbufferProgramIndex,
	// deferred shading
	deferredShadingProgramIndex,
	// deferred lighting
	deferredLightPassProgramIndex,
	deferredCompositeProgramIndex,

	depthOfFieldCompositeProgramIndex,

	drawCurveProgram, 
	testSolidColorProgramIndex,

	//-----------------------------
	GLSLProgramCount
};

enum GLSLCommonUniformIndex
{
	unif_mvp,

	unif_lightColor,
	unif_lightPos,
	unif_eyePos,

	unif_dm,
	unif_sm,

	unif_pixelSizeInv,
	unif_img,
	unif_img1,
	unif_img2,
	unif_img3,
	unif_img4,

	// deferred rendering
	unif_modelMat,
	unif_viewprojMat,
	unif_atlasMat,
	unif_normalScale,
	unif_img_position,
	unif_img_normal,
	unif_img_texcoord,
	unif_img_depth,
	unif_img_light_diffuse,
	unif_img_light_specular,

	unif_waypoint,
	unif_waypointCount,
	unif_curveMode,
	unif_useWaypoints,
	unif_color,

	//-----------------------------
	GLSLCommonUniformCount
};


// framebuffer objects (FBOs)
enum FBOIndex
{
	// scene
	sceneFBO,

	// bloom
	brightFBO_d2,
	hblurFBO_d2,
	vblurFBO_d2,
	hblurFBO_d4,
	vblurFBO_d4,
	hblurFBO_d8,
	vblurFBO_d8,
	compositeFBO,

	// deferred rendering
	gbufferSceneFBO,
	// deferred shading
	deferredShadingFBO,
	// deferred lighting
	lightPassFBO,
	deferredLightingCompositeFBO,

	depthOfFieldOutputFBO,

	curvesFBO,

	// shadow map
	shadowFBO,

	//-----------------------------
	fboCount
};

enum DrawCurveMode
{
	CURVE_LINES,
	CURVE_BEZIER,
	CURVE_CATMULLROM,
	CURVE_HERMITE,
};

// test vertex buffers for built-in and loaded primitive data
// use same model index for VAO and VBO (since VAOs only work 
//	if the respective VBO has been created and associated)
enum ModelIndex
{
	// built-in models
	axesModel,
	fsqModel,
	skyboxModel, sphere8x6Model, sphere32x24Model,

	// loaded models
	sphereLowResObjModel,
	sphereHiResObjModel,

	pointModel,

	//-----------------------------
	modelCount
};