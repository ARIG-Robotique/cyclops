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

	cv::Point3d GetPanelPosition(int index) const
	{
		const double panelY=-1.037;
		const double panelGroupSeparationX=1.0, panelSeparationX=0.225;
		const double ExpectedZ = 0.104;
		int i = (index/3)-1; int j = (index%3)-1;
		double panelX=i*panelGroupSeparationX+j*panelSeparationX;
		return cv::Point3d(panelX, panelY, ExpectedZ);
	}

	virtual cv::Affine3d GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
		std::map<int, ArucoCornerArray> &ReprojectedCorners) override;

	virtual bool ShouldBeDisplayed(uint64_t Tick) const override
	{
		(void) Tick;
		return true;
	}

	virtual std::vector<ObjectData> ToObjectData() const override;

	virtual std::vector<std::vector<cv::Point3d>> GetPointsOfInterest() const override;
};
