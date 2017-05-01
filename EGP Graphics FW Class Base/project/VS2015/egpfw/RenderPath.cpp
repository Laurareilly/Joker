#include "RenderPath.h"
#include "egpfw/egpfw.h"

RenderPath::RenderPath()
{
}

RenderPath::~RenderPath()
{
}

void RenderPath::addRenderPass(const RenderPass& pass)
{
	mPasses.push_back(pass);
}

void RenderPath::addRenderPass(RenderPass&& pass)
{
	mPasses.push_back(std::move(pass));
}

void RenderPath::addRenderPasses(std::initializer_list<RenderPass> passes)
{
	for (auto iter = passes.begin(); iter != passes.end(); ++iter)
		mPasses.push_back(*iter);
}

void RenderPath::clearAllPasses()
{
	mPasses.clear();
}

void RenderPath::render()
{
	//Activate and draw each of our passes.

	for (auto pass : mPasses)
	{
		pass.activate();
		pass.sendData();
		egpDrawActiveVAO();
	}
}
