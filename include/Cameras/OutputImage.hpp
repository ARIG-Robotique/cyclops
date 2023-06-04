#pragma once

#include <opencv2/core.hpp>


//Father class to represent any object capable of outputting a 2D image
class OutputImage
{

public:
	OutputImage(/* args */);
	~OutputImage();

	virtual void SetFrame(cv::UMat& frame);

	virtual void GetFrame(cv::UMat& frame);

	virtual void GetOutputFrame(cv::UMat& frame, cv::Rect window);
};

cv::UMat ConcatCameras(std::vector<OutputImage*> Cameras, int NumCams);
