#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

//Represents all the aruco tags seen by a single camera, as well as it's data need to recontruct objects in 3D space
struct CameraArucoData
{
	cv::Affine3d CameraTransform;
	cv::Mat CameraMatrix;
	cv::Mat DistanceCoefficients;
	std::vector<int> TagIDs;
	std::vector<std::vector<cv::Point2f>> TagCorners;
	class Camera *SourceCamera;

};
