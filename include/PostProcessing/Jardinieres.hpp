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
	};
	std::array<StockStatus, 6> Stocks;
public:
	PostProcessJardinieres(CDFRExternal* InOwner);

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
