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

const auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
unique_ptr<aruco::ArucoDetector> GlobalDetector, POIDetector;

void MakeDetectors()
{
	const int adaptiveThreshConstant = 20;
	if (!GlobalDetector.get())
	{
		auto params = aruco::DetectorParameters();
		//enable corner refine only if aruco runs at native resolution
		params.cornerRefinementMethod = GetReductionFactor() >= 1.0 ? aruco::CORNER_REFINE_CONTOUR : aruco::CORNER_REFINE_NONE;
		params.useAruco3Detection = false;
		params.adaptiveThreshConstant = adaptiveThreshConstant;

		//params.minMarkerDistanceRate *= mulfac;
		auto refparams = aruco::RefineParameters();
		GlobalDetector = make_unique<aruco::ArucoDetector>(dict, params, refparams);
	}
	if (!POIDetector.get())
	{
		auto params = aruco::DetectorParameters();
		//enable corner refine only if aruco runs at native resolution
		params.cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
		params.useAruco3Detection = false;
		params.adaptiveThreshConstant = adaptiveThreshConstant;

		//params.minMarkerDistanceRate *= mulfac;
		auto refparams = aruco::RefineParameters();
		POIDetector = make_unique<aruco::ArucoDetector>(dict, params, refparams);
	}
}

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

Point2f ComputeMean(const vector<Point2f> &Points)
{
	Point2f mean(0,0);
	for (size_t j = 0; j < Points.size(); j++)
	{
		mean += Points[j];
	}
	mean /= (int)Points.size();
	return mean;
}

int DetectArucoSegmented(const CameraImageData &InData, CameraFeatureData& OutData, const vector<Rect> &Segments, aruco::ArucoDetector* Detector)
{
	size_t NumSegments = Segments.size();
	if (NumSegments == 0)
	{
		return 0;
	}
	
	vector<vector<vector<Point2f>>> corners;
	vector<vector<int>> ids;
	corners.resize(NumSegments);
	ids.resize(NumSegments);
	parallel_for_(Range(0, NumSegments), 
	[&InData, &Segments, &corners, &ids, NumSegments, Detector]
	(Range InRange)
	{
		//Range InRange(0, numpois);
		for (int poiidx = InRange.start; poiidx < InRange.end; poiidx++)
		{
			auto &thispoirect = Segments[poiidx];
			auto &cornerslocal = corners[poiidx];
			auto &idslocal = ids[poiidx];
			Detector->detectMarkers(InData.Image(thispoirect), cornerslocal, idslocal);
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

	size_t NumDetectionsBefore = OutData.ArucoCorners.size();
	size_t NumDetectionsThis = 0;
	for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
	{
		NumDetectionsThis += ids[poiidx].size();
	}
	size_t MaxDetectionsAfter = NumDetectionsThis + NumDetectionsBefore;
	vector<int> accumulations;
	vector<Point2f> means;
	accumulations.resize(NumDetectionsBefore, 1);
	accumulations.reserve(MaxDetectionsAfter);
	means.resize(NumDetectionsBefore, Point2f(0,0));
	means.reserve(MaxDetectionsAfter);
	for (size_t i = 0; i < NumDetectionsBefore; i++)
	{
		means[i] = ComputeMean(OutData.ArucoCorners[i]);
	}
	const float SameWindowSize = 4;
	const Rect2f SameThreshold(-SameWindowSize/2,-SameWindowSize/2,SameWindowSize,SameWindowSize);
	OutData.ArucoCorners.reserve(MaxDetectionsAfter);
	OutData.ArucoIndices.reserve(MaxDetectionsAfter);
	for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
	{
		size_t numdetslocal = ids[poiidx].size();
		if (numdetslocal ==0)
		{
			continue;
		}
		vector<vector<Point2f>> &CornersLocal = corners[poiidx];
		vector<int> &IDsLocal = ids[poiidx];
		for (size_t PotentialIdx = 0; PotentialIdx < numdetslocal; PotentialIdx++)
		{
			bool found = false;
			Point2f mean = ComputeMean(CornersLocal[PotentialIdx]);
			for (size_t PresentIdx = 0; PresentIdx < OutData.ArucoCorners.size(); PresentIdx++)
			{
				if (OutData.ArucoIndices[PresentIdx] != IDsLocal[PotentialIdx])
				{
					continue;
				}
				Point2f &meanother = means[PresentIdx];
				Point2f diff = mean-meanother;
				if (diff.inside(SameThreshold))
				{
					found = true;
					//mean the mean
					meanother = (meanother*accumulations[PresentIdx] + mean) / (accumulations[PresentIdx]+1);
					for (size_t corneridx = 0; corneridx < OutData.ArucoCorners[PresentIdx].size(); corneridx++)
					{
						//mean the corner locations
						OutData.ArucoCorners[PresentIdx][corneridx] = (OutData.ArucoCorners[PresentIdx][corneridx]*accumulations[PresentIdx] + CornersLocal[PotentialIdx][corneridx]) / (accumulations[PresentIdx]+1);
					}
					accumulations[PresentIdx]++;
					cout << "Merging aruco at " << mean << endl;
					break;
				}
			}
			if (found)
			{
				continue;
			}
			means.push_back(mean);
			OutData.ArucoIndices.push_back(IDsLocal[PotentialIdx]);
			OutData.ArucoCorners.push_back(CornersLocal[PotentialIdx]);
			accumulations.push_back(1);
		}
	}
	OutData.ArucoCornersReprojected.resize(OutData.ArucoIndices.size());
	copy(Segments.begin(), Segments.end(), back_inserter(OutData.ArucoSegments));
	return NumDetectionsThis;
}

void CleanArucoDetections(CameraFeatureData& OutData)
{
	OutData.ArucoCorners.clear();
	OutData.ArucoIndices.clear();
	OutData.ArucoCornersReprojected.clear();
	OutData.ArucoSegments.clear();
}

int DetectArucoSegmented(const CameraImageData &InData, CameraFeatureData& OutData, int MaxArucoSize, Size Segments)
{
	MakeDetectors();
	CleanArucoDetections(OutData);

	Size framesize = InData.Image.size();
	vector<Rect> ROIs;
	ROIs.reserve(Segments.area());
	Size2d cutsize(
		framesize.width /(double)Segments.width, 
		framesize.height/(double)Segments.height
	);
	Size2d OverlappedFrameSize
	{
		(double) framesize.width  + (Segments.width -1) * MaxArucoSize,
		(double) framesize.height + (Segments.height-1) * MaxArucoSize,
	};
	Size2d segmentsize(
		OverlappedFrameSize.width /Segments.width,
		OverlappedFrameSize.height/Segments.height
	);
	for (int xseg = 0; xseg < Segments.width; xseg++)
	{
		double xstart = (segmentsize.width-MaxArucoSize)*xseg;
		xstart = max<double>(0, xstart);
		double xend = xstart + segmentsize.width;
		xend = min<double>(framesize.width, xend);
		for (int yseg = 0; yseg < Segments.height; yseg++)
		{
			double ystart = (segmentsize.height-MaxArucoSize)*yseg;
			ystart = max<double>(0, ystart);
			double yend = ystart + segmentsize.height;
			yend = min<double>(framesize.height, yend);
			ROIs.emplace_back(xstart, ystart, xend - xstart, yend - ystart);
		}
	}
	return DetectArucoSegmented(InData, OutData, ROIs, GlobalDetector.get());
}

int DetectAruco(const CameraImageData &InData, CameraFeatureData& OutData)
{
	MakeDetectors();
	CleanArucoDetections(OutData);

	Size framesize = InData.Image.size();
	Size rescaled = GetArucoReduction();
	UMat GrayFrame = PreprocessArucoImage(InData.Image);

	UMat ResizedFrame;
	if (rescaled != framesize)
	{
		assert(rescaled.height <= framesize.height && rescaled.width <= framesize.width);
		resize(GrayFrame, ResizedFrame, rescaled);
	}
	else
	{
		ResizedFrame = GrayFrame;
	}

	vector<vector<Point2f>> &corners = OutData.ArucoCorners;
	vector<int> &IDs = OutData.ArucoIndices;

	try
	{
		GlobalDetector->detectMarkers(ResizedFrame, corners, IDs);
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

vector<Rect> GetPOIRects(const vector<vector<Point3d>> &POIs, Size framesize, Affine3d CameraTransform, InputArray CameraMatrix, InputArray distCoeffs)
{
	size_t numpois = POIs.size();
	if (numpois == 0)
	{
		return {};
	}
	vector<Rect> poirects;
	poirects.reserve(numpois);
	auto InvCamTransform = CameraTransform.inv();
	auto rvec = InvCamTransform.rvec();
	auto tvec = InvCamTransform.translation();
	for (size_t poiidx = 0; poiidx < numpois; poiidx++)
	{
		vector<Point2d> reprojected;
		projectPoints(POIs[poiidx], rvec, tvec, CameraMatrix, distCoeffs, reprojected);
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
	return poirects;
}

int DetectArucoPOI(const CameraImageData &InData, CameraFeatureData& OutData, const vector<vector<Point3d>> &POIs)
{
	MakeDetectors();
	Size framesize = InData.Image.size();
	vector<Rect> poirects = GetPOIRects(POIs, framesize, OutData.CameraTransform, InData.CameraMatrix, InData.DistanceCoefficients);

	return DetectArucoSegmented(InData, OutData, poirects, POIDetector.get());
}