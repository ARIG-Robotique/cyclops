#include "Communication/ProcessedTypes.hpp"

#include <Cameras/ImageTypes.hpp>

void CameraFeatureData::Clear()
{
    ArucoIndices.clear();
    ArucoCorners.clear();
    ArucoCornersReprojected.clear();

    YoloIndices.clear();
    YoloCorners.clear();
}

void CameraFeatureData::CopyEssentials(const CameraImageData &source)
{
    CameraName = source.CameraName;
    CameraMatrix = source.CameraMatrix;
    DistanceCoefficients = source.DistanceCoefficients;
    FrameSize = cv::Size(source.Image.cols, source.Image.rows);
}