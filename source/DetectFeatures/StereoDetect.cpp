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

	//static auto Matcher = StereoSGBM::create(-128, 256, block_size, 8*block_size*block_size, 32*block_size*block_size, 0, 0, 5, 200, 2);
	static auto Matcher = StereoBM::create(16, 21);
	size_t num_lenses = InData.lenses.size();
	if (num_lenses != 2)
	{
		return;
	}
	Point3d LensMeanCenter, LensMeanForward, LensMeanX;
	Point2d OpticalMeanCenter;
	for (size_t i = 0; i < num_lenses; i++)
	{
		LensMeanCenter = Vec3d(LensMeanCenter) + InData.lenses[i].LensPosition.translation();
		LensMeanForward = Vec3d(LensMeanForward) + Vec3d(GetAxis(InData.lenses[i].LensPosition.rotation(), 2).val);
		LensMeanX = Vec3d(LensMeanX) + Vec3d(GetAxis(InData.lenses[i].LensPosition.rotation(), 0).val);
		OpticalMeanCenter.x += InData.lenses[i].CameraMatrix.at<double>(0,2);
		OpticalMeanCenter.y += InData.lenses[i].CameraMatrix.at<double>(1,2);
	}
	LensMeanCenter /= (double)num_lenses;
	LensMeanForward /= (double)num_lenses;
	LensMeanX /= (double)num_lenses;
	OpticalMeanCenter /= (double)num_lenses;
	const LensSettings &left_lens = InData.lenses[0], &right_lens = InData.lenses[1];
	Mat OptimalMatrix = left_lens.CameraMatrix.clone();
	OptimalMatrix.at<double>(0,0) /= 1.5;
	OptimalMatrix.at<double>(1,1) /= 1.5;
	OptimalMatrix.at<double>(0,2) = OpticalMeanCenter.x;
	OptimalMatrix.at<double>(1,2) = OpticalMeanCenter.y;
	vector<Point3d> LensFocusPoints = {LensMeanCenter + LensMeanForward*5.0, LensMeanCenter + LensMeanForward*5.0+LensMeanX};
	vector<vector<Point2d>> ConvergencePoints(num_lenses);
	for (size_t i = 0; i < num_lenses; i++)
	{
		vector<Point2d> &LocalConvergencePoints = ConvergencePoints[i];
		projectPoints(LensFocusPoints, InData.lenses[i].LensPosition.rotation(), InData.lenses[i].LensPosition.translation(), 
			OptimalMatrix, Mat(), LocalConvergencePoints);
	}
	
	
	vector<UMat> undistorted(num_lenses);
	UMat gray;
	cvtColor(InData.Image, gray, COLOR_BGR2GRAY);
	for (size_t i = 0; i < num_lenses; i++)
	{
		OptimalMatrix.at<double>(0,2) = ConvergencePoints[i][0].x;
		OptimalMatrix.at<double>(1,2) = ConvergencePoints[i][0].y;
		undistort(gray(InData.lenses[i].ROI), undistorted[i], InData.lenses[i].CameraMatrix, InData.lenses[i].distanceCoeffs, OptimalMatrix);
		//cout << "Lens " << i << " right vector deviation from mean is " << 1-(LensMeanX.ddot(Vec3d(GetAxis(InData.lenses[i].LensPosition.rotation(), 0).val))) << endl;
	}
	
	UMat disparity, d_norm;
	//cout << "CV_8UC1 = " << (int)CV_8UC1 << "; image channels = " << InData.Image.channels() <<endl;
	Matcher->setTextureThreshold(1);
	Matcher->setUniquenessRatio(2);
	Matcher->setPreFilterType(StereoBM::PREFILTER_XSOBEL);
	//Matcher->setPreFilterCap(50);
	Matcher->compute(undistorted[0], undistorted[1], disparity);
	normalize(disparity, d_norm, 255, 0, NORM_MINMAX, CV_8UC1);
	//imshow("L", undistorted[0]);
	//imshow("R", undistorted[1]);
	imshow("Disparity", d_norm);
	waitKey(1);
}