#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

struct CameraFeatureData
{
	std::string CameraName;
	cv::Mat CameraMatrix;
	cv::Mat DistanceCoefficients;

	cv::Affine3d CameraTransform;

	std::vector<std::vector<cv::Point2f>> ArucoCorners, ArucoCornersReprojected;
	std::vector<int> ArucoIndices;

	std::vector<cv::Rect2f> YoloCorners;
	std::vector<int> YoloIndices;
};