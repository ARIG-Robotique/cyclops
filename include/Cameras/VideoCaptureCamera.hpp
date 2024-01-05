#pragma once

#include <iostream>
#include <string>   // for strings
#include <memory>
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/core/affine.hpp>


#include <Cameras/Camera.hpp>
#include <Cameras/ImageTypes.hpp>


class VideoCaptureCamera : public Camera
{


private:
	//capture using classic api
	std::unique_ptr<cv::VideoCapture> feed;

public:

	VideoCaptureCamera(std::shared_ptr<VideoCaptureCameraSettings> InSettings)
		:Camera(InSettings)
	{
	}

	~VideoCaptureCamera()
	{
		
	}

	//Start the camera
	virtual bool StartFeed() override;

	//Lock a frame to be captured at this time
	//This allow for simultaneous capture
	virtual bool Grab() override;

	//Retrieve or read a frame
	virtual bool Read() override;

};