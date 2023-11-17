#pragma once

#include <opencv2/core.hpp>
#include <Cameras/ImageTypes.hpp>


//Father class to represent any object capable of outputting a 2D image
class ImageSource
{

public:
	ImageSource(/* args */);
	~ImageSource();

	virtual void SetFrame(CameraImageData& frame, bool Distorted);

	virtual void GetFrame(CameraImageData& frame, bool Distorted);
};
