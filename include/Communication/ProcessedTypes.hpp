#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <ArucoPipeline/ArucoTypes.hpp>

struct YoloDetection
{
	cv::Rect2f Corners;
	int Class;
	float Confidence;
};

struct CameraFeatureData
{
	std::string CameraName; 		//Filled by CopyEssentials from CameraImageData
	cv::Mat CameraMatrix; 			//Filled by CopyEssentials from CameraImageData
	cv::Mat DistanceCoefficients; 	//Filled by CopyEssentials from CameraImageData

	cv::Affine3d CameraTransform; 	//Filled by CopyEssentials from CameraImageData
	cv::Size FrameSize; 			//Filled by CopyEssentials from CameraImageData

	std::vector<ArucoCornerArray> ArucoCorners, 		//Filled by ArucoDetect
		ArucoCornersReprojected; 						//Cleared by ArucoDetect, Filled by ObjectTracker
	std::vector<int> ArucoIndices; 						//Filled by ArucoDetect
	std::vector<cv::Rect> ArucoSegments;				//Filled by ArucoDetect

	std::vector<YoloDetection> YoloDetections; 	//Filled by YoloDetect

	void Clear();
	void CopyEssentials(const struct CameraImageData &source);
};