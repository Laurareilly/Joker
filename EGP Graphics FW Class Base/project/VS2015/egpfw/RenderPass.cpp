#include "RenderPass.h"
#include <stdexcept>

RenderPass::RenderPass(egpFrameBufferObjectDescriptor* fbos, egpProgram* programs)
{
	mFBOArray = fbos;
	mProgramArray = programs;

	//Set our activation data to the defaults 
	//i.e., if someone activates us now, we won't do any damage (or anything at all).
	mProgram = GLSLProgramCount;
	mPipelineStage = fboCount;
	mAssociatedVAO = nullptr;
}

void RenderPass::addUniform(const render_pass_uniform_int& i)
{
	mIntUniforms.push_back(i);
}

void RenderPass::addUniform(const render_pass_uniform_float& f)
{
	mFloatUniforms.push_back(f);
}

void RenderPass::addUniform(const render_pass_uniform_float_complex& f)
{
	mComplexFloatUniforms.push_back(f);
}

void RenderPass::addUniform(const render_pass_uniform_float_matrix& fm)
{
	mFloatMatrixUniforms.push_back(fm);
}

void RenderPass::addTexture(const RenderPassTextureData& t)
{
	mTextures.push_back(t);
}

void RenderPass::setVAO(egpVertexArrayObjectDescriptor* vao)
{
	mAssociatedVAO = vao;
}

void RenderPass::setProgram(GLSLProgramIndex p)
{
	mProgram = p;
}

void RenderPass::setPipelineStage(FBOIndex i)
{
	mPipelineStage = i;
}

void RenderPass::sendData() const
{
	//Loop through all of our data and send it to OpenGL using the appropriate EGP helper functions.

	for (auto target : mColorTargets)
		egpfwBindColorTargetTexture(mFBOArray + target.fboIndex, target.glBinding, target.targetIndex);

	for (auto target : mDepthTargets)
		egpfwBindDepthTargetTexture(mFBOArray + target.fboIndex, target.glBinding);

	for (auto data : mIntUniforms)
		egpSendUniformInt(data.location, data.type, data.count, data.values);
	
	for (auto data : mFloatUniforms)
		egpSendUniformFloat(data.location, data.type, data.count, data.values);

	for (auto data : mComplexFloatUniforms)
		egpSendUniformFloat(data.location, data.type, data.count, fetchVals(data.values).data());

	for (auto data : mFloatMatrixUniforms)
		egpSendUniformFloatMatrix(data.location, UNIF_MAT4, data.count, data.transpose, data.value->m);

	for (auto data : mTextures)
	{
		glActiveTexture(data.textureLane);
		glBindTexture(data.textureType, data.textureHandle);
	}
}

void RenderPass::activate() const
{
	//Activate our program (if we have one)
	if (mProgram != GLSLProgramCount)
		egpActivateProgram(mProgramArray + mProgram);

	//Activate our target FBO (if we have one)
	if (mPipelineStage != fboCount)
		egpfwActivateFBO(mFBOArray + mPipelineStage);

	//Activate our target VAO (if we have one)
	if (mAssociatedVAO != nullptr)
		egpActivateVAO(mAssociatedVAO);
}
