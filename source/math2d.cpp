#include "math2d.hpp"

using namespace cv;
using namespace std;

Size ScaleToFit(Size original, Size target)
{
	Size &insize = target;
	Size &bssize = original;
	int wmax = bssize.width*insize.height/bssize.height;
	wmax = min(insize.width, wmax);
	int hmax = bssize.height*insize.width/bssize.width;
	hmax = min(insize.height, hmax);
	
	Size outsize(wmax, hmax);
	return outsize;
}

cv::Rect ScaleToFit(cv::Size original, cv::Rect target)
{
	Size targetsz = ScaleToFit(original, target.size());
	Size szdiff = target.size()-targetsz;
	Rect roi(szdiff.width/2 + target.x,szdiff.height/2 + target.y, targetsz.width, targetsz.height);
	return roi;
}

Size2d GetCameraFOV(Size Resolution, Mat CameraMatrix)
{
	if (CameraMatrix.size().width != 3 || CameraMatrix.size().height != 3)
	{
		return Size2d(M_PI/2, M_PI/2);
	}
	double fx = CameraMatrix.at<double>(0,0), fy = CameraMatrix.at<double>(1,1);
	double fovx = 2*atan(Resolution.width/(2*fx)), fovy = 2*atan(Resolution.height/(2*fy));
	return Size2d(fovx, fovy);
} 