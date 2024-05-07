#pragma once

#include <vector>
#include <array>
#include <memory>
#include <future>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>

class CDFRInternal
{
public:
	struct InternalResult
	{
		CameraFeatureData FeatureData;
		std::vector<ObjectData> ObjData;
	};
private:
	InternalResult Process(CameraImageData InData, CDFRTeam Team);

public:
	CDFRInternal();
	~CDFRInternal();

	std::shared_future<InternalResult> Inject(CameraImageData &InData, CDFRTeam Team);
};
