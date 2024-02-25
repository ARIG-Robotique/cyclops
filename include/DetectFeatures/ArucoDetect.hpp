#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>

cv::UMat PreprocessArucoImage(cv::UMat Source);

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData);

int DetectArucoPOI(const CameraImageData &InData, CameraFeatureData& OutData, const std::vector<std::vector<cv::Point3d>> POIs);