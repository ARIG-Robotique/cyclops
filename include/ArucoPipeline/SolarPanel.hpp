#pragma once

#include <ArucoPipeline/TrackedObject.hpp>

#include <array>

class SolarPanel : public TrackedObject
{
private:
	std::array<double, 9> PanelRotations; //Solar panels, X increasing (so going from blue to yellow side)
	std::array<cv::Point3d, 9> PanelPositions;
public:
	SolarPanel();

	virtual cv::Affine3d GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
		std::map<int, std::vector<cv::Point2f>> &ReprojectedCorners) override;

	virtual bool ShouldBeDisplayed(uint64_t Tick) override
	{
		(void) Tick;
		return true;
	}

	virtual std::vector<ObjectData> ToObjectData() const override;
};
