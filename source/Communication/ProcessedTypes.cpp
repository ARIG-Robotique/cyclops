#include "Communication/ProcessedTypes.hpp"

#include <Cameras/ImageTypes.hpp>

void CameraFeatureData::Clear()
{
    ArucoIndices.clear();
    ArucoCorners.clear();
    ArucoCornersReprojected.clear();

    YoloDetections.clear();
}

void CameraFeatureData::CopyEssentials(const CameraImageData &source)
{
    CameraName = source.CameraName;
    CameraMatrix = source.CameraMatrix;
    DistanceCoefficients = source.DistanceCoefficients;
    FrameSize = source.Image.size();
}