#pragma once
#include "egpfw/egpfw/egpfwFrameBuffer.h"
#include "egpfw/egpfw/utils/egpfwVertexBufferUtils.h"
#include "egpfw/egpfw/utils/egpfwShaderProgramUtils.h"
#include "RenderPath.h"
#include <vector>

class RenderNetgraph
{
	private:
		RenderPath mInternalRenderPath;

		int* mTextureProgramUniforms;
		egpVertexArrayObjectDescriptor* mQuadModel;
		egpFrameBufferObjectDescriptor* mFBOArray;
		egpProgram* mProgramArray;
		
		std::vector<FBOTargetColorTexture> mFBOsToDraw;
		std::vector<cbmath::mat4> mQuadMVPs;


		/**
		 * \brief Using the current list of FBO targets, creates an array of matrices to be used for their transformation. */
		void generateMVPs();
		/**
		 * \brief Sets up the internal render path based on the current list of FBO targets. */
		void setupRenderPasses();

	public:
		RenderNetgraph(egpFrameBufferObjectDescriptor* fboArray, egpVertexArrayObjectDescriptor* quad, egpProgram* programs, int* textureProgramUniforms);
		~RenderNetgraph() = default;

		void clearFBOList();
		void addFBO(FBOTargetColorTexture fbo);
		void addFBOs(std::initializer_list<FBOTargetColorTexture> fbos);
		
		FBOTargetColorTexture getFBOAtIndex(unsigned i);

		void render();
};
