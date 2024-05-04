#pragma once

#include "Cameras/CameraManager.hpp"

class CameraManagerV4L2 : public CameraManager
{
private:
	CameraStartType Start;
	std::string Filter;
	bool AllowNoCalib;
public:
	CameraManagerV4L2(CameraStartType InStart, std::string InFilter, bool InAllowNoCalib = false)
		:CameraManager(), Start(InStart), Filter(InFilter), AllowNoCalib(InAllowNoCalib)
	{

	}

	virtual ~CameraManagerV4L2()
	{

	}

	//Check if the name can fit the filter to blacklist or whitelist certain cameras based on name
	static bool DeviceInFilter(v4l2::devices::DEVICE_INFO device, std::string Filter);

	//Gather calibration info and start setting (fps, resolution, method...) before starting the camera
	static VideoCaptureCameraSettings DeviceToSettings(v4l2::devices::DEVICE_INFO device, CameraStartType Start);

	static std::vector<VideoCaptureCameraSettings> autoDetectCameras(CameraStartType Start, std::string Filter, bool silent = true);

	protected:
	virtual void ScanWorker() override;
};
