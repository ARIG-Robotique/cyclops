#pragma once

#include <ArucoPipeline/TrackedObject.hpp>
#include <optional>

//A cube with 4 tags, one on each side
class TopTracker : public TrackedObject
{
private:
	std::optional<double> ExpectedHeight;
	bool Robot;
public:
	TopTracker(int MarkerIdx, double MarkerSize, std::string InName, std::optional<double> InExpectedHeight, bool InRobot);
	~TopTracker();

	virtual cv::Affine3d GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
		std::map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners) override;
		
	virtual std::vector<ObjectData> ToObjectData() const override;
};