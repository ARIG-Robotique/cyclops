#include "DetectFeatures/ArucoDetect.hpp"

#include <iostream> // for standard I/O
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/imgproc.hpp>

#include <math2d.hpp>
#include <math3d.hpp>

#include <GlobalConf.hpp>

using namespace cv;
using namespace std;

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData)
{
	Size framesize = InData.Image.size();
	Size rescaled = GetArucoReduction();

	UMat GrayFrame;
	int numchannels = InData.Image.channels();
	switch (numchannels)
	{
	case 3:
		cvtColor(InData.Image, GrayFrame, COLOR_BGR2GRAY);
		break;
	case 1:
		GrayFrame = InData.Image;
		break;
	
	default:
		cout << "depth " << numchannels << endl; 
		assert(false);
		break;
	}

	UMat ResizedFrame;
	if (rescaled != framesize)
	{
		assert(rescaled.height <= framesize.height && rescaled.width <= framesize.width);
		resize(GrayFrame, ResizedFrame, rescaled);
	}
	else
	{
		ResizedFrame = InData.Image;
	}

	vector<vector<Point2f>> &corners = OutData.ArucoCorners;
	vector<int> &IDs = OutData.ArucoIndices;
	corners.clear();
	IDs.clear();
	OutData.ArucoCornersReprojected.clear();

	GetArucoDetector().detectMarkers(ResizedFrame, corners, IDs);

	if (framesize != rescaled)
	{
		Size2d scalefactor((double)framesize.width/rescaled.width, (double)framesize.height/rescaled.height);
		float reductionFactors = GetReductionFactor();

		//rescale corners to full image position
		for (int j = 0; j < corners.size(); j++)
		{
			for (size_t k = 0; k < 4; k++)
			{
				Point2f& corner = corners[j][k];
				corner.x *= scalefactor.width;
				corner.y *= scalefactor.height;
			}
		}

		for (int ArucoIdx = 0; ArucoIdx < IDs.size(); ArucoIdx++)
		{
			Size window = Size(reductionFactors, reductionFactors);
			cornerSubPix(GrayFrame, corners[ArucoIdx], window, Size(-1,-1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.01));
		}
	}
	OutData.ArucoCornersReprojected.resize(corners.size(), {});
	return IDs.size();
}