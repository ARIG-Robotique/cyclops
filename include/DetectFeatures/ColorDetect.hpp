#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>

void DetectColor(const CameraImageData &InData, CameraFeatureData& OutData);

cv::Mat MultiThreshold(const cv::UMat &Image, const std::vector<cv::Vec3b> &Colors, cv::Vec3b tolerance, float dilateAmount, float erodeAmount);