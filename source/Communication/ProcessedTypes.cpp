#include "Communication/ProcessedTypes.hpp"

#include <Cameras/ImageTypes.hpp>

void CameraFeatureData::Clear()
{
	ArucoIndices.clear();
	ArucoCorners.clear();
	ArucoCornersReprojected.clear();
	ArucoSegments.clear();

	YoloDetections.clear();
}

void CameraFeatureData::CopyEssentials(const CameraImageData &source, int lens)
{
	CameraName = source.CameraName;
	CameraMatrix = source.lenses[lens].CameraMatrix;
	DistanceCoefficients = source.lenses[lens].distanceCoeffs;
	FrameSize = source.lenses[lens].ROI.size();
	CameraTransform = source.lenses[lens].LensPosition;
}