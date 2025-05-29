#include <DetectFeatures/StereoDetect.hpp>

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cvconfig.h>
#include <opencv2/ximgproc.hpp>

#include <Misc/math3d.hpp>

using namespace cv;
using namespace std;

Ptr<StereoBM> Matcher;
Ptr<ximgproc::DisparityWLSFilter> Filter;
Ptr<StereoMatcher> RightMatcher;

void MakeMatchers()
{
	if (!Matcher)
	{
		Matcher = StereoBM::create(16*8, 21);
		//Matcher->setTextureThreshold(1);
		//Matcher->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
		//Matcher->setUniquenessRatio(2);
	}
	if (!Filter && Matcher)
	{
		Filter = ximgproc::createDisparityWLSFilter(Matcher);
	}
	if (!RightMatcher && Matcher)
	{
		RightMatcher = ximgproc::createRightMatcher(Matcher);
	}
}


void DetectStereo(const CameraImageData &InData, CameraFeatureData& OutData)
{
	static const int block_size = 11;

	MakeMatchers();
	size_t num_lenses = InData.lenses.size();
	if (num_lenses != 2)
	{
		return;
	}
	assert(!InData.Distorted);
	
	
	Mat left_disparity, right_disparity, disparity, d_norm;
	//cout << "CV_8UC1 = " << (int)CV_8UC1 << "; image channels = " << InData.Image.channels() <<endl;
	//Matcher->setPreFilterCap(50);
	UMat left_image = InData.Image(InData.lenses[0].ROI), right_image = InData.Image(InData.lenses[1].ROI);
	Matcher->compute(left_image, right_image, left_disparity);
	RightMatcher->compute(right_image, left_image, right_disparity);
	Filter->filter(left_disparity, left_image, disparity, right_disparity, Rect(), right_image);
	d_norm = disparity * 50; 
	//normalize(disparity, d_norm, 255, 0, NORM_MINMAX, CV_8UC1);
	//imshow("L", undistorted[0]);
	//imshow("R", undistorted[1]);
	imshow("Disparity", d_norm);
	waitKey(1);
}