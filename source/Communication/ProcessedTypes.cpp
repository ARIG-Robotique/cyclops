#include "Communication/ProcessedTypes.hpp"

#include <Cameras/ImageTypes.hpp>

void LensFeatureData::Clear()
{
	ArucoIndices.clear();
	ArucoCorners.clear();
	ArucoCornersReprojected.clear();
	StereoReprojected.clear();
	YoloDetections.clear();
}

void CameraFeatureData::Clear()
{
	for (auto &&i : Lenses)
	{
		i.Clear();
	}
	ArucoSegments.clear();
	ArucoCornersStereo.clear();
	ArucoIndicesStereo.clear();
	
}

void CameraFeatureData::CopyEssentials(const CameraImageData &source)
{
	CameraName = source.CameraName;
	Lenses.resize(source.lenses.size());
	for (size_t i = 0; i < source.lenses.size(); i++)
	{
		LensFeatureData &feat = Lenses[i];
		auto &lens = source.lenses[i];
		feat.CameraMatrix = lens.CameraMatrix;
		feat.DistanceCoefficients = lens.distanceCoeffs;
		feat.LensTransform = lens.LensPosition;
		feat.ROI = lens.ROI;
	}
	FrameSize = source.Image.size();
}