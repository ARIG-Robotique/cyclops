#pragma once

#include <PostProcessing/PostProcess.hpp>
#include <array>
#include <vector>

class PostProcessJardinieres : public PostProcess
{
	struct StockStatus
	{
		CDFRTeam team;
		int NumPlants = 6;
		std::string name;
		std::vector<cv::Vec3d> Corners;
		cv::Vec2d BuzzingPoint;
		double BuzzingRadius = 0.35/2.0;
		bool Contacting = false, ContactThisTick = false;
		ObjectData::TimePoint LastContactStart;
		ObjectData::TimePoint LastContactEnd;
		ObjectData::Clock::duration TimeSpentContacting;
	};
	std::array<StockStatus, 6> Stocks;
public:
	PostProcessJardinieres(CDFRExternal* InOwner);

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
