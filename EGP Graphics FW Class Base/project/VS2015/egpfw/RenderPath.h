#pragma once
#include <vector>
#include "RenderPass.h"

class RenderPath
{
	private:
		std::vector<RenderPass> mPasses;

	public:
		RenderPath();
		~RenderPath();

		void addRenderPass(const RenderPass& pass);
		void addRenderPass(RenderPass&& pass);
		void addRenderPasses(std::initializer_list<RenderPass> passes);
		void clearAllPasses();

		/**
		 * \brief Activates and renders every pass in our collection. */
		void render();
};
