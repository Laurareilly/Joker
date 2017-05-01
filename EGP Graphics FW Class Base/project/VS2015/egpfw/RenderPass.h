#pragma once
#include "render_enums.h"
#include "egpfw/egpfw/egpfwFrameBuffer.h"
#include <egpfw/egpfw/utils/egpfwVertexBufferUtils.h>
#include "RenderPassData.h"

class RenderPass
{
	public:

	private:
		int mProgram;
		int mPipelineStage;
		egpFrameBufferObjectDescriptor* mFBOArray;
		egpProgram* mProgramArray;

		std::vector<render_pass_uniform_int> mIntUniforms;
		std::vector<render_pass_uniform_float> mFloatUniforms;
		std::vector<render_pass_uniform_float_complex> mComplexFloatUniforms;
		std::vector<render_pass_uniform_float_matrix> mFloatMatrixUniforms;

		std::vector<FBOTargetColorTexture> mColorTargets;
		std::vector<FBOTargetDepthTexture> mDepthTargets;

		std::vector<RenderPassTextureData> mTextures;
		egpVertexArrayObjectDescriptor* mAssociatedVAO;

		template <typename T>
		static std::vector<T> fetchVals(std::vector<T*>& refs);

	public:
		/**
		* \brief Create a RenderPass.
		* \param fbos Pointer to the global FBO array.
		* \param programs Pointer to the global GLSLProgram array. */
		RenderPass(egpFrameBufferObjectDescriptor* fbos, egpProgram* programs);
		~RenderPass() = default;

		/**
		 * \brief Set the VAO that this RenderPass should activate.
		 * \param vao Pointer to the appropriate VAO. If set to nullptr, the RenderPass will use whatever VAO was last active. */
		void setVAO(egpVertexArrayObjectDescriptor* vao);
		/**
		 * \brief Set the GLSL program that this RenderPass should use. 
		 * \param p Index of the program to use. If set to GLSLProgramCount, the RenderPass will use whatever program was last active. */
		void setProgram(GLSLProgramIndex p);
		/**
		* \brief Set the FBO that this RenderPass should use.
		* \param i Index of the FBO to use. If set to fboCount, the RenderPass will use whatever FBO was last active. */
		void setPipelineStage(FBOIndex i);

		void addUniform(const render_pass_uniform_int& i);
		void addUniform(const render_pass_uniform_float& f);
		void addUniform(const render_pass_uniform_float_complex& f);
		void addUniform(const render_pass_uniform_float_matrix& fm);

		void addColorTarget(const FBOTargetColorTexture& ct) { mColorTargets.push_back(ct); }
		void addDepthTarget(const FBOTargetDepthTexture& dt) { mDepthTargets.push_back(dt); }
		void addTexture(const RenderPassTextureData& t);

		/**
		 * \brief Prepare to render by sending all of this RenderPass's data to OpenGL using the egp helper functions. */
		void sendData() const;
		/**
		 * \brief Prepare to render by activating our GLSL Program, FBO, and VAO. */
		void activate() const;
};

/**
 * \brief Used with complex pass data. Fetches simple data at arbitrary memory addresses and builds them into one final buffer. */
template <typename T>
inline std::vector<T> RenderPass::fetchVals(std::vector<T*>& refs)
{
	std::vector<T> result;
	result.reserve(refs.size());

	for (auto iter : refs)
		result.push_back(*iter);

	return result;
}
