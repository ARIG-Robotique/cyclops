#include <DetectFeatures/StereoDetect.hpp>

#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Misc/math3d.hpp>

using namespace cv;
using namespace std;

void DetectStereo(const CameraImageData &InData, CameraFeatureData& OutData)
{
	static const int block_size = 11;

	#if 0
	static auto Matcher = StereoSGBM::create(-128, 256, block_size, 8*block_size*block_size, 32*block_size*block_size, 0, 0, 5, 200, 2);
	#else
	static auto Matcher = StereoBM::create(16*4, 21);
	//Matcher->setTextureThreshold(1);
	//Matcher->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
	//Matcher->setUniquenessRatio(2);
	#endif
	size_t num_lenses = InData.lenses.size();
	if (num_lenses != 2)
	{
		return;
	}
	assert(!InData.Distorted);
	
	
	UMat disparity, d_norm;
	//cout << "CV_8UC1 = " << (int)CV_8UC1 << "; image channels = " << InData.Image.channels() <<endl;
	//Matcher->setPreFilterCap(50);
	Matcher->compute(
		InData.Image(InData.lenses[0].ROI), 
		InData.Image(InData.lenses[1].ROI), 
		disparity);
	normalize(disparity, d_norm, 255, 0, NORM_MINMAX, CV_8UC1);
	//imshow("L", undistorted[0]);
	//imshow("R", undistorted[1]);
	imshow("Disparity", d_norm);
	waitKey(1);
}