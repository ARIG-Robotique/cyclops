#pragma once

#include <PostProcessing/PostProcess.hpp>
#include <array>

class PostProcessStockPlants : public PostProcess
{
	struct StockStatus
	{
		ObjectData::TimePoint LastTouched;
		int NumPlants = 6;
		std::string name;
		cv::Vec2d position;
		double radius = 0.25/2;
	};
	std::array<StockStatus, 6> Stocks;
public:
	PostProcessStockPlants(CDFRExternal* InOwner);

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
