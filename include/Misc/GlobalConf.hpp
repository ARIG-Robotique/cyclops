#pragma once

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/core/affine.hpp>
#include <filesystem>

//Defines all global config parameters, and also reads the config file.

enum class CameraStartType;

enum class RunType
{
	Normal,
	Simulate
};

struct CaptureConfig
{
	int StartType; //See CameraStartType in Misc/ImageTypes.hpp . Chooses method to use to start the camera
	cv::Size FrameSize; //resolution
	float ReductionFactor; //factor to downscale image before aruco detection
	int CaptureFramerate;
	int FramerateDivider;
	std::string filter; //filter to block or allow certain cameras. If camera name contains the filter string, it's allowed. If the filter string starts with a !, the filter is inverted
};

extern bool RecordVideo;

std::filesystem::path GetAssetsPath();

RunType GetRunType();

const cv::aruco::ArucoDetector& GetArucoDetector();

cv::Size GetFrameSize();

int GetCaptureFramerate();

CaptureConfig GetCaptureConfig();

CameraStartType GetCaptureMethod();

//list of downscales to be done to the aruco detections
float GetReductionFactor();

//list of resolutions in the end
cv::Size GetArucoReduction();

cv::UMat& GetArucoImage(int id);

struct InternalCameraConfig
{
	std::string CameraName;
	cv::Affine3d LocationRelative;
};

std::vector<InternalCameraConfig>& GetInternalCameraPositionsConfig();

struct CalibrationConfig
{
	float SquareSideLength; //mm
	cv::Size NumIntersections; //number of square intersections, ex for a chess board is 7x7
	float ReprojectionErrorOffset; //score computation : (numimages^NumImagePower)/(ReprojectionError + Offset)
	float NumImagePower;
	cv::Size2d SensorSize; //only used for stats
};

const CalibrationConfig& GetCalibrationConfig();