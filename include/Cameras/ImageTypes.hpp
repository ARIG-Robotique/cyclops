#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <thirdparty/list-devices.hpp>

//All the settings needed to start a camera, in the palm of your hand...
struct CameraSettings
{
	

	//General data
	//Resolution of the frame to be captured
	cv::Size Resolution;
	//Framerate
	uint8_t Framerate;
	//Framerate divider : you can set 60fps but only sample 1 of 2 frames to have less latency and less computation
	uint8_t FramerateDivider;

	

	//FOV and center
	cv::Mat CameraMatrix;
	//Distortion
	cv::Mat distanceCoeffs;

	CameraSettings()
	:Resolution(-1,-1), Framerate(0), FramerateDivider(1)
	{}

	virtual ~CameraSettings(){};

	bool IsValidCalibration();
	bool IsValid();
};

//Camera API and configuration selection
enum class CameraStartType
{
	ANY = 0,
	GSTREAMER_CPU = 1
};

struct VideoCaptureCameraSettings : public CameraSettings
{
	//which api should be used ?
	CameraStartType StartType;

	//data from v4l2 about the device
	v4l2::devices::DEVICE_INFO DeviceInfo;

	//Initialisation string
	cv::String StartPath;
	//API to be used for opening
	int ApiID;
};

struct CameraImageData
{
	std::string CameraName;
	cv::UMat Image;
	cv::Mat CameraMatrix;
	cv::Mat DistanceCoefficients;
	bool Distorted;
};
