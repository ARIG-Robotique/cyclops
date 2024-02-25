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

UMat PreprocessArucoImage(UMat Source)
{
	int numchannels = Source.channels();
	if (numchannels == 1)
	{
		return Source;
	}
	assert(numchannels == 3);
	if (0)
	{
		UMat hsvImage; vector<UMat> hsvplanes;
		cvtColor(Source, hsvImage, COLOR_BGR2HSV);
		split(hsvImage, hsvplanes);
		UMat grayout;
		if (0)
		{
			addWeighted(hsvplanes[1], 0.5, hsvplanes[2], 1, 0, grayout);
		}
		else
		{
			add(hsvplanes[1], hsvplanes[2], grayout, noArray());
		}
		return grayout;
	}
	else
	{
		UMat grayimage;
		cvtColor(Source, grayimage, COLOR_BGR2GRAY);
		return grayimage;
	}
}

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData)
{
	Size framesize = InData.Image.size();
	Size rescaled = GetArucoReduction();

	static aruco::ArucoDetector* ArucoDet(nullptr);
	if (!ArucoDet)
	{
		auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
		auto params = aruco::DetectorParameters();
		//enable corner refine only if aruco runs at native resolution
		params.cornerRefinementMethod = GetReductionFactor() >= 1.0 ? aruco::CORNER_REFINE_CONTOUR : aruco::CORNER_REFINE_NONE;
		params.useAruco3Detection = false;
		//params.adaptiveThreshConstant = 20;

		//params.minMarkerDistanceRate *= mulfac;
		auto refparams = aruco::RefineParameters();
		ArucoDet = new aruco::ArucoDetector(dict, params, refparams);
	}


	UMat GrayFrame = PreprocessArucoImage(InData.Image);

	UMat ResizedFrame;
	if (rescaled != framesize)
	{
		assert(rescaled.height <= framesize.height && rescaled.width <= framesize.width);
		resize(GrayFrame, ResizedFrame, rescaled);
	}
	else
	{
		InData.Image.copyTo(ResizedFrame);
	}

	vector<vector<Point2f>> &corners = OutData.ArucoCorners;
	vector<int> &IDs = OutData.ArucoIndices;
	corners.clear();
	IDs.clear();
	OutData.ArucoCornersReprojected.clear();

	try
	{
		ArucoDet->detectMarkers(ResizedFrame, corners, IDs);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return 0;
	}

	if (framesize != rescaled)
	{
		Size2d scalefactor((double)framesize.width/rescaled.width, (double)framesize.height/rescaled.height);
		float reductionFactors = GetReductionFactor();

		//rescale corners to full image position
		for (size_t j = 0; j < corners.size(); j++)
		{
			for (size_t k = 0; k < 4; k++)
			{
				Point2f& corner = corners[j][k];
				corner.x *= scalefactor.width;
				corner.y *= scalefactor.height;
			}
		}

		for (size_t ArucoIdx = 0; ArucoIdx < IDs.size(); ArucoIdx++)
		{
			Size window = Size(reductionFactors, reductionFactors);
			cornerSubPix(GrayFrame, corners[ArucoIdx], window, Size(-1,-1), TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 100, 0.01));
		}
	}
	OutData.ArucoCornersReprojected.resize(corners.size(), {});
	return IDs.size();
}

int DetectArucoPOI(const CameraImageData &InData, CameraFeatureData& OutData, const vector<vector<Point3d>> POIs)
{

	size_t numpois = POIs.size();
	if (numpois == 0)
	{
		return 0;
	}
	Size framesize = InData.Image.size();
	vector<Rect> poirects;
	poirects.reserve(numpois);
	auto InvCamTransform = OutData.CameraTransform.inv();
	auto rvec = InvCamTransform.rvec();
	auto tvec = InvCamTransform.translation();
	for (size_t poiidx = 0; poiidx < numpois; poiidx++)
	{
		vector<Point2d> reprojected;
		projectPoints(POIs[poiidx], rvec, tvec, InData.CameraMatrix, InData.DistanceCoefficients, reprojected);
		int top=framesize.height,bottom=0,left=framesize.width,right=0;
		for (auto p : reprojected)
		{
			left = min<int>(left, p.x);
			right = max<int>(right, p.x);
			top = min<int>(top, p.y);
			bottom = max<int>(bottom, p.y);
		}
		left = max(0, left);
		right = min(right, framesize.width);
		top = max(0, top);
		bottom = min(bottom, framesize.height);
		int width = max(right-left,0);
		int height = max(bottom-top,0);
		if (width*height < 32)
		{
			continue;
		}
		poirects.emplace_back(left, top, width, height);
	}
	numpois = poirects.size();
	static aruco::ArucoDetector* POIDet(nullptr);
	if (!POIDet)
	{
		auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
		auto params = aruco::DetectorParameters();
		//enable corner refine only if aruco runs at native resolution
		params.cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
		params.useAruco3Detection = false;
		//params.adaptiveThreshConstant = 20;

		//params.minMarkerDistanceRate *= mulfac;
		auto refparams = aruco::RefineParameters();
		POIDet = new aruco::ArucoDetector(dict, params, refparams);
	}
	vector<vector<vector<Point2f>>> corners;
	vector<vector<int>> ids;
	corners.resize(numpois);
	ids.resize(numpois);
	parallel_for_(Range(0, numpois), 
	[&InData, &poirects, &corners, &ids, numpois]
	(Range InRange)
	{
		//Range InRange(0, numpois);
		for (int poiidx = InRange.start; poiidx < InRange.end; poiidx++)
		{
			auto &thispoirect = poirects[poiidx];
			auto &cornerslocal = corners[poiidx];
			auto &idslocal = ids[poiidx];
			POIDet->detectMarkers(InData.Image(thispoirect), cornerslocal, idslocal);
			for (auto &rect : cornerslocal)
			{
				for (auto &point : rect)
				{
					point.x+=thispoirect.x;
					point.y+=thispoirect.y;
				}
			}
		}
	});

	size_t numdets = 0;
	for (size_t poiidx = 0; poiidx < numpois; poiidx++)
	{
		size_t numdetslocal = ids[poiidx].size();
		if (numdetslocal ==0)
		{
			continue;
		}
		copy(corners[poiidx].begin(), corners[poiidx].end(), back_inserter(OutData.ArucoCorners));
		copy(ids[poiidx].begin(), ids[poiidx].end(), back_inserter(OutData.ArucoIndices));
	}
	OutData.ArucoCornersReprojected.resize(OutData.ArucoIndices.size());
	return numdets;
}