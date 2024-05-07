#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <vector>
#include <set>
#include <functional>

class CDFRExternal;

class PostProcess
{
protected:
	CDFRExternal* Owner;
public:
	PostProcess(CDFRExternal* InOwner);
	virtual ~PostProcess();
	
	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects);
};