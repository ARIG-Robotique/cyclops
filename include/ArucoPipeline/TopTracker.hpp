#pragma once

#include <ArucoPipeline/TrackedObject.hpp>

//A cube with 4 tags, one on each side
class TopTracker : public TrackedObject
{
private:
	double ExpectedHeight;
public:
	TopTracker(int MarkerIdx, double MarkerSize, std::string InName, double InExpectedHeight);
	~TopTracker();

	virtual cv::Affine3d GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
		std::map<int, ArucoCornerArray> &ReprojectedCorners) override;
		
	virtual std::vector<ObjectData> ToObjectData() const override;
};