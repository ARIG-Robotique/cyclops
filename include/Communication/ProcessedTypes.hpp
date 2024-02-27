#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

struct CameraFeatureData
{
	std::string CameraName; 		//Filled by CopyEssentials from CameraImageData
	cv::Mat CameraMatrix; 			//Filled by CopyEssentials from CameraImageData
	cv::Mat DistanceCoefficients; 	//Filled by CopyEssentials from CameraImageData

	cv::Affine3d CameraTransform; 	//Filled by CopyEssentials from CameraImageData
	cv::Size FrameSize; 			//Filled by CopyEssentials from CameraImageData

	std::vector<std::vector<cv::Point2f>> ArucoCorners, //Filled by ArucoDetect
		ArucoCornersReprojected; 						//Cleared by ArucoDetect, Filled by ObjectTracker
	std::vector<int> ArucoIndices; 						//Filled by ArucoDetect
	std::vector<cv::Rect> ArucoSegments;

	std::vector<cv::Rect2f> YoloCorners; 	//Filled by YoloDetect
	std::vector<int> YoloIndices; 			//Filled by YoloDetect

	void Clear();
	void CopyEssentials(const struct CameraImageData &source);
};