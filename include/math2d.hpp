#pragma once

#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

template <class T>
T wraptwopi(T in)
{
	in = fmod(in, M_PI*2);
	if (in > M_PI)
	{
		in -= M_PI*2;
	}
	if (in < -M_PI)
	{
		in += M_PI*2;
	}
	return in;
}

//Gives the new size to use to fit inside the target size while keeping aspect ratio
cv::Size ScaleToFit(cv::Size original, cv::Size target);

//Give the new rect to fit in the target rect while keeping aspect ratio
cv::Rect ScaleToFit(cv::Size original, cv::Rect target);

//Get the FOV of a camera from it's matrix, in radians
cv::Size2d GetCameraFOV(cv::Size Resolution, cv::Mat CameraMatrix);