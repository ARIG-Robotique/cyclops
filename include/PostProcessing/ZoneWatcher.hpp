#pragma once

#include <PostProcessing/PostProcess.hpp>
#include <array>
#include <atomic>

class PostProcessZone : public PostProcess
{
	struct ZoneStatus
	{
		std::string name;
		cv::Rect2d position;
		double radius = 0.25;
		bool IsStock;
		ObjectData::TimePoint LastContactStart, LastContactEnd;
		ObjectData::Clock::duration TimeSpentContacting;
		bool Contacting=false, ContactThisTick=false;
		std::array<std::atomic<int>, 10> depthBuckets;
	};
	std::array<ZoneStatus, 18> Zones;
public:
	PostProcessZone(CDFRExternal* InOwner);

	virtual void Reset() override;

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
