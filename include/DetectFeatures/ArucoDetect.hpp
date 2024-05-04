#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>
#include <opencv2/core.hpp>

cv::UMat PreprocessArucoImage(cv::UMat Source);

std::vector<cv::Rect> GetPOIRects(const std::vector<std::vector<cv::Point3d>> &POIs, cv::Size framesize, 
	cv::Affine3d CameraTransform, cv::InputArray CameraMatrix, cv::InputArray distCoeffs);

int DetectAruco(CameraImageData InData, CameraFeatureData *OutData);

int DetectArucoSegmented(CameraImageData InData, CameraFeatureData *OutData, int MaxArucoSize, cv::Size Segments);

int DetectArucoPOI(CameraImageData InData, CameraFeatureData *OutData, const std::vector<std::vector<cv::Point3d>> &POIs);