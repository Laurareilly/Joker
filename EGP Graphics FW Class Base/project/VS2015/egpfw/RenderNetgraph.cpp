#include "RenderNetgraph.h"

void RenderNetgraph::generateMVPs()
{
	//Remove the old matricies
	mQuadMVPs.clear();

	//Do some magic number stuff to calculate our scale and position.
	float scale = 0.5f / mFBOsToDraw.size();

	const float XPOS_START = 0.0f + scale / 2.0f;
	const float XPOS_END = 1.0f + scale / 2.0f;

	//Some magic numbers that look pretty good most of time.
	float yPos = (-1.0f + scale * 1.25f);
	if (yPos < -0.75f)
		yPos = -0.75f;

	//Loop through and generate a transformation matrix for each quad.
	for (size_t i = 0; i < mFBOsToDraw.size(); i++)
	{
		float t = static_cast<float>(i) / static_cast<float>(mFBOsToDraw.size());
		float xPos = XPOS_START + (XPOS_END - XPOS_START) * t;

		mQuadMVPs.push_back(cbmath::makeTranslation4(xPos, yPos, 0.0f) * cbmath::makeScale4(scale));
	}
}

void RenderNetgraph::setupRenderPasses()
{
	//Remove the old data and generate positions.
	mInternalRenderPath.clearAllPasses();
	generateMVPs();

	for (size_t i = 0; i < mFBOsToDraw.size(); i++)
	{
		//Create a pass for each quad.
		RenderPass p(mFBOArray, mProgramArray);

		//Set program and VAO, but don't provide a pipeline stage. Prevents changing the currently active FBO.
		p.setProgram(testTextureProgramIndex);
		p.setVAO(mQuadModel);
		p.addColorTarget(mFBOsToDraw[i]);
		p.addUniform(render_pass_uniform_float_matrix(mTextureProgramUniforms[unif_mvp], 1, 0, &mQuadMVPs[i]));

		//Add it to our internal render path.
		mInternalRenderPath.addRenderPass(p);
	}
}

RenderNetgraph::RenderNetgraph(egpFrameBufferObjectDescriptor* fboArray, egpVertexArrayObjectDescriptor* quad, egpProgram* programs, int* textureProgramUniforms)
{
	mFBOArray = fboArray;
	mQuadModel = quad;
	mProgramArray = programs;
	mTextureProgramUniforms = textureProgramUniforms;
}

void RenderNetgraph::clearFBOList()
{
	mFBOsToDraw.clear();
	mInternalRenderPath.clearAllPasses();
}

void RenderNetgraph::addFBO(FBOTargetColorTexture fbo)
{
	fbo.glBinding = 0; //we always want it to be at lane 0.
	mFBOsToDraw.push_back(fbo);

	setupRenderPasses();
}

void RenderNetgraph::addFBOs(std::initializer_list<FBOTargetColorTexture> fbos)
{
	for (auto fbo : fbos)
	{
		fbo.glBinding = 0; //we always want it to be at lane 0.
		mFBOsToDraw.push_back(fbo);
	}

	setupRenderPasses();
}

FBOTargetColorTexture RenderNetgraph::getFBOAtIndex(unsigned i)
{
	if (i >= mFBOsToDraw.size() - 1)
		i = mFBOsToDraw.size() - 1;

	return mFBOsToDraw[i];
}

void RenderNetgraph::render() 
{
	//Ready to render? Just call our internal render path.
	mInternalRenderPath.render();
}
