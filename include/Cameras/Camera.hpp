#pragma once


#include <iostream>
#include <string>   // for strings
#include <vector>
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudacodec.hpp>
#endif

#include "Cameras/OutputImage.hpp"
#include "Cameras/ImageTypes.hpp"
#include "TrackedObjects/TrackedObject.hpp"

class Camera;
struct CameraArucoData;

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
class Camera : public OutputImage, public TrackedObject
{
protected:
	//config
	CameraSettings Settings;

	bool HasUndistortionMaps;
	
	#ifdef WITH_CUDA
	cv::cuda::GpuMat UndistMap1, UndistMap2;
	#else
	cv::UMat UndistMap1, UndistMap2;
	#endif
public:
	int errors;

protected:
	//frame buffer, increases fps but also latency
	BufferedFrame FrameBuffer;

public:
	//status
	bool connected;

public:

	Camera(CameraSettings InSettings)
		:TrackedObject(), Settings(InSettings),
		HasUndistortionMaps(false),
		errors(0),
		connected(false),
		FrameBuffer()
	{}

	~Camera()
	{}

protected:
	void RegisterError();
	void RegisterNoError();
public:

	//Get the settings used to start this camera
	virtual CameraSettings GetCameraSettings();

	//Set the settings to used for this camera
	virtual bool SetCameraSetting(CameraSettings InSettings);

	virtual bool SetCalibrationSetting(cv::Mat CameraMatrix, cv::Mat DistanceCoefficients);

	virtual void GetCameraSettingsAfterUndistortion(cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

	//Get the status of a buffer (read, aruco'ed, 3D-ed...)
	virtual BufferStatus GetStatus();

	//Start the camera
	virtual bool StartFeed();

	//Lock a frame to be capture at this time
	//This allow for simultaneous capture
	virtual bool Grab();

	//Retrieve or read a frame
	virtual bool Read();

	virtual void Undistort();

	virtual void GetFrameUndistorted(cv::UMat& frame);

	virtual void Calibrate(std::vector<std::vector<cv::Point3f>> objectPoints,
	std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<std::string> imagePaths, cv::Size imageSize,
	cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs);

	virtual void GetFrame(cv::UMat& frame) override;

	virtual void GetOutputFrame(cv::UMat& frame, cv::Rect window) override;

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;

	virtual cv::Affine3d GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError) override;

	//Create lower-resolution copies of the frame to be used in aruco detection
	virtual void RescaleFrames();

	//detect markers using the lower resolutions and improve accuracy using the higher-resolution image
	virtual void detectMarkers(cv::aruco::ArucoDetector& Detector);

	//Gather detected markers in screen space
	virtual bool GetMarkerData(CameraArucoData& CameraData);

	//Used to debug reprojected location of markers in 2D debug (direct)
	void SetMarkerReprojection(int MarkerIndex, const std::vector<cv::Point2d> &Corners);
};