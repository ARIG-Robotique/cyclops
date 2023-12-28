#pragma once

#include <cmath>
#include <vector>
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

template<class T> 
T remap(T xmin, T xmax, T ymin, T ymax, T x)
{
	return (x-xmin)*(ymax-ymin)/(xmax-xmin)+ymin;
}

template<class T> 
T remapw(T xmin, T xwidth, T ymin, T ywidth, T x)
{
	return (x-xmin)*(ywidth)/(xwidth)+ymin;
}

//Gives the new size to use to fit inside the target size while keeping aspect ratio
cv::Size ScaleToFit(cv::Size original, cv::Size target);

//Give the new rect to fit in the target rect while keeping aspect ratio
cv::Rect ScaleToFit(cv::Size original, cv::Rect target);

std::vector<cv::Rect> DistributeViewports(cv::Size ImageSize, cv::Size ViewportSize, int NumSources);

template<class T>
cv::Point_<T> ImageRemap(cv::Rect_<T> InputMap, cv::Rect_<T> OutputMap, cv::Point_<T> X)
{
	return cv::Point_<T>(
		remapw(InputMap.x, InputMap.width, OutputMap.x, OutputMap.width, X.x),
		remapw(InputMap.y, InputMap.height, OutputMap.y, OutputMap.height, X.y)
	);
}

//Get the FOV of a camera from it's matrix, in radians
cv::Size2d GetCameraFOV(cv::Size Resolution, cv::Mat CameraMatrix);