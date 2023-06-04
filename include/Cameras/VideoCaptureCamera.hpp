#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>


#include "Cameras/Camera.hpp"
#include "data/ImageTypes.hpp"


class VideoCaptureCamera : public Camera
{


private:
	//capture using classic api
	cv::VideoCapture* feed;

public:

	VideoCaptureCamera(CameraSettings InSettings)
		:Camera(InSettings)
	{
	}

	~VideoCaptureCamera()
	{
		if (connected)
		{
			delete feed;
		}
	}

	//Start the camera
	virtual bool StartFeed() override;

	//Lock a frame to be captured at this time
	//This allow for simultaneous capture
	virtual bool Grab() override;

	//Retrieve or read a frame
	virtual bool Read() override;

};