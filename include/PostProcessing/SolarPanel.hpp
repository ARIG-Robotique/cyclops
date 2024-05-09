#pragma once

#include <PostProcessing/PostProcess.hpp>

class PostProcessSolarPanel : public PostProcess
{
public:
	PostProcessSolarPanel(CDFRExternal* InOwner)
		:PostProcess(InOwner)
	{}

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
