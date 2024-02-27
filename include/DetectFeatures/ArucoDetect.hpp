#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>

cv::UMat PreprocessArucoImage(cv::UMat Source);

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData);

int DetectArucoSegmented(const CameraImageData &InData, CameraFeatureData& OutData, int MaxArucoSize, cv::Size Segments);

int DetectArucoPOI(const CameraImageData &InData, CameraFeatureData& OutData, const std::vector<std::vector<cv::Point3d>> &POIs);