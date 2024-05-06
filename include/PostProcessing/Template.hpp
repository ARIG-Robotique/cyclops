#pragma once

#include <PostProcessing/PostProcess.hpp>

class PostProcessTemplate : public PostProcess
{
public:
	PostProcessTemplate(CDFRExternal* InOwner)
		:PostProcess(InOwner)
	{}

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
