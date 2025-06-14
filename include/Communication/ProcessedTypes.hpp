#pragma once

#include <string>
#include <vector>
#include <optional>
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
	cv::Affine3d CameraToLens;				//Camera to lens transform
	cv::Affine3d WorldToLens;
	cv::Mat CameraMatrix; 					//Filled by CopyEssentials from CameraImageData
	cv::Mat ProjectionMatrix;				//Filled on undistortion, a 4x3 camera matrix if needed
	cv::Mat DistanceCoefficients; 			//Filled by CopyEssentials from CameraImageData
	cv::Rect ROI;							//Region of interest in the source image

	std::vector<ArucoCornerArray> ArucoCorners; 						//Filled by ArucoDetect
	mutable std::vector<ArucoCornerArray> ArucoCornersReprojected; 		//Cleared by ArucoDetect, Filled by ObjectTracker
	std::vector<int> ArucoIndices; 										//Filled by ArucoDetect
	std::vector<bool> StereoReprojected;								//True if the aruco tags was projected to 3D using multilens

	std::vector<YoloDetection> YoloDetections; 	//Filled by YoloDetect

	void Clear();
};

struct DepthData
{
	cv::Mat DepthMap;
	cv::Mat CameraMatrix, DistortionCoefficients;
	cv::Affine3d CameraToDepth; //camera to reference lens used for depth;
};


struct CameraFeatureData
{
	std::string CameraName; 		//Filled by CopyEssentials from CameraImageData
	std::vector<cv::Rect> ArucoSegments;				//Filled by ArucoDetect

	cv::Affine3d WorldToCamera; 	//Filled by CopyEssentials from CameraImageData, World to camera
	cv::Size FrameSize; 			//Filled by CopyEssentials from CameraImageData

	std::vector<LensFeatureData> Lenses;

	std::vector<std::vector<cv::Point3d>> ArucoCornersStereo; //Marker corners, in lens 0 frame of reference
	std::vector<int> ArucoIndicesStereo;

	std::optional<DepthData> Depth;

	void Clear();
	void CopyEssentials(const struct CameraImageData &source);
};