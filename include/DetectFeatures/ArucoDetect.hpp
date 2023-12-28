#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>

cv::UMat PreprocessArucoImage(cv::UMat Source);

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData);