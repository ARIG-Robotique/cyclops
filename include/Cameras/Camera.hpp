#pragma once


#include <iostream>
#include <string>   // for strings
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#include <Cameras/ImageSource.hpp>
#include <Cameras/ImageTypes.hpp>
#include <ArucoPipeline/TrackedObject.hpp>

class Camera;
struct CameraImageData;

template<class CameraClass>
std::vector<CameraClass*> StartCameras(std::vector<CameraSettings> CameraSettings)
{
	std::vector<CameraClass*> Cameras;
	for (int i = 0; i < CameraSettings.size(); i++)
	{
		Camera* cam = new CameraClass(CameraSettings[i]);
		Cameras.push_back((CameraClass*)cam);
		if(!Cameras[i]->StartFeed())
		{
			std::cerr << "ERROR! Unable to open camera " << Cameras[i]->GetCameraSettings().DeviceInfo.device_description << std::endl;
		}
	}
	return Cameras;
}

//Base class for cameras, it's a wrapper
//Has location and functions to get an image
//Handles everything related to the image it has captured
//Will never throw an error
//Made to increase an error counter if a communication error occurs
//Can be destroyed
//Should not have any aruco tags attached
class Camera : public ImageSource, public TrackedObject
{
protected:
	//config
	std::shared_ptr<CameraSettings> Settings;
	std::string Name;
	bool HasUndistortionMaps;
	
	cv::UMat UndistMap1, UndistMap2;
	cv::UMat LastFrameDistorted, LastFrameUndistorted;
public:
	int errors;
	//status
	bool connected;
	bool grabbed;
	std::chrono::time_point<std::chrono::steady_clock> captureTime;

public:

	Camera(std::shared_ptr<CameraSettings> InSettings)
		:TrackedObject(), Settings(InSettings),
		HasUndistortionMaps(false),
		errors(0),
		connected(false)
	{}

	~Camera()
	{}

protected:
	void RegisterError();
	void RegisterNoError();
public:

	std::string GetName()
	{
		return Name;
	}

	//Get the settings used to start this camera
	//Please do not modify...
	virtual const CameraSettings* GetCameraSettings();

	//Set the settings to used for this camera
	virtual bool SetCameraSetting(std::shared_ptr<CameraSettings> InSettings);

	virtual bool SetCalibrationSetting(cv::Mat CameraMatrix, cv::Mat DistanceCoefficients);

	virtual void GetCameraSettingsAfterUndistortion(cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

	//Start the camera
	virtual bool StartFeed();

	//Lock a frame to be capture at this time
	//This allow for simultaneous capture
	virtual bool Grab();

	//Retrieve or read a frame
	virtual bool Read();

	virtual void Undistort();

	virtual CameraImageData GetFrame(bool Distorted) override;

	virtual std::vector<ObjectData> ToObjectData() const override;
};