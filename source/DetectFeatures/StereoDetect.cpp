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

#define USE_FILTERS 1

void MakeMatchers()
{
	if (!Matcher)
	{
		Matcher = StereoBM::create(16*2, 21);
		//Matcher->setTextureThreshold(1);
		//Matcher->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
		//Matcher->setUniquenessRatio(2);
	}
	#if USE_FILTERS
	if (!Filter && Matcher)
	{
		Filter = ximgproc::createDisparityWLSFilter(Matcher);
	}
	if (!RightMatcher && Matcher)
	{
		RightMatcher = ximgproc::createRightMatcher(Matcher);
	}
	#endif
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
	#if USE_FILTERS
	RightMatcher->compute(right_image, left_image, right_disparity);
	Filter->filter(left_disparity, left_image, disparity, right_disparity, Rect(), right_image);
	#else
	disparity = left_disparity;
	#endif
	DepthData depth;
	Affine3d Lens1ToLens2 = InData.lenses[0].CameraToLens.inv() * InData.lenses[1].CameraToLens;
	Point3d L1L2 = Lens1ToLens2.translation();
	reprojectImageTo3D(disparity, depth.DepthMap, InData.DisparityToDepth, false);
	depth.DepthMap /= L1L2.x*1.5; //not sure about that
	depth.CameraMatrix = InData.lenses[0].CameraMatrix;
	depth.CameraToDepth = InData.lenses[0].CameraToLens;
	depth.DistortionCoefficients = InData.lenses[0].distanceCoeffs;
	OutData.Depth = depth;
	Point2i middle = disparity.size()/2;
	cout << "Disparity in the middle " << disparity.at<int16_t>(middle) << ", type " << disparity.type() <<
	", 3D in the middle " << depth.DepthMap.at<Point3f>(middle) <<endl;
	//normalize(disparity, d_norm, 255, 0, NORM_MINMAX, CV_8UC1);
	//imshow("L", undistorted[0]);
	//imshow("R", undistorted[1]);
	d_norm = depth.DepthMap * 8.0;
	imshow("Disparity", d_norm);
	waitKey(1);
}