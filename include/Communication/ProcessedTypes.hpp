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

struct ArucoFeatureData
{
	ArucoCornerArray Corners, CornersReprojected;
	int index = -1;
	bool StereoReprojected = false;
};

struct LensFeatureData
{
	cv::Affine3d LensTransform;     //Camera to lens transform
	cv::Mat CameraMatrix; 			//Filled by CopyEssentials from CameraImageData
	cv::Mat DistanceCoefficients; 	//Filled by CopyEssentials from CameraImageData
	cv::Rect ROI;

	std::vector<ArucoCornerArray> ArucoCorners, 		//Filled by ArucoDetect
		ArucoCornersReprojected; 						//Cleared by ArucoDetect, Filled by ObjectTracker
	std::vector<int> ArucoIndices; 						//Filled by ArucoDetect
	std::vector<bool> StereoReprojected;				//True if the aruco tags was projected to 3D using multilens

	std::vector<YoloDetection> YoloDetections; 	//Filled by YoloDetect

	void Clear();
};

struct CameraFeatureData
{
	std::string CameraName; 		//Filled by CopyEssentials from CameraImageData
	std::vector<cv::Rect> ArucoSegments;				//Filled by ArucoDetect

	cv::Affine3d CameraTransform; 	//Filled by CopyEssentials from CameraImageData, World to camera
	cv::Size FrameSize; 			//Filled by CopyEssentials from CameraImageData

	std::vector<LensFeatureData> Lenses;

	std::vector<std::vector<cv::Point3d>> ArucoCornersStereo; //Marker corners, in lens 0 frame of reference
	std::vector<int> ArucoIndicesStereo;

	void Clear();
	void CopyEssentials(const struct CameraImageData &source);
};