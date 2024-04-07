#pragma once

#include <thread>
#include <vector>
#include <array>
#include <memory>
#include <future>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>

class CDFRInternal
{
private:
	bool direct, v3d;
	ObjectTracker BlueTracker, YellowTracker;
public:
	CDFRInternal();
	~CDFRInternal();

	std::future<CameraFeatureData> Inject(CameraImageData &InData, CDFRTeam Team);
};
