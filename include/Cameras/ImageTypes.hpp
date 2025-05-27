#pragma once

#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/affine.hpp>
#include <thirdparty/list-devices.hpp>

struct LensSettings
{
	cv::Rect2i ROI;
	//FOV and center
	cv::Mat CameraMatrix;
	//Distortion
	cv::Mat distanceCoeffs;

	cv::Affine3d CameraToLens;

	bool IsValid() const;
};

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

	bool WantUndistortion;
	double UndistortFocalLengthMuliply;
	bool IsMonochrome;

	//single lens settings, for side-by-side cams
	std::vector<LensSettings> Lenses;


	//Frame number at which to toggle camera position lock (camera always starts unlocked, used for simulation)
	std::vector<unsigned int> CameraLockToggles;

	CameraSettings()
	:Resolution(-1,-1), Framerate(0), FramerateDivider(1), WantUndistortion(false), UndistortFocalLengthMuliply(1.0), IsMonochrome(false)
	{}

	virtual ~CameraSettings(){};

	bool IsValidCalibration() const;
	bool IsValid() const;

	bool IsMono() const
	{
		return Lenses.size() == 1;
	}

	bool IsStereo() const
	{
		return Lenses.size() == 2;
	}
};

//Camera API and configuration selection
enum class CameraStartType
{
	ANY = 0,
	GSTREAMER_CPU,
	PLAYBACK //playback from a file
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

	std::vector<LensSettings> lenses;
	std::chrono::steady_clock::time_point GrabTime;
	bool Distorted;
	bool Valid = false;
};
