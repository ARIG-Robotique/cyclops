#include "DetectFeatures/ArucoDetect.hpp"

#include <iostream> // for standard I/O
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/imgproc.hpp>

#include <Misc/math2d.hpp>
#include <Misc/math3d.hpp>

#include <Misc/GlobalConf.hpp>

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

Point2f ComputeMean(const ArucoCornerArray &Points)
{
	Point2f mean(0,0);
	for (size_t j = 0; j < Points.size(); j++)
	{
		mean += Points[j];
	}
	mean /= (int)Points.size();
	return mean;
}

int DetectArucoSegmented(CameraImageData InData, CameraFeatureData *OutData, const vector<vector<Rect>> &Segments, aruco::ArucoDetector* Detector)
{
	size_t num_lenses = InData.lenses.size();

	
	vector<vector<vector<ArucoCornerArray>>> corners;
	vector<vector<vector<int>>> ids;
	corners.resize(num_lenses);
	ids.resize(num_lenses);
	size_t num_segments_total = 0;
	for (size_t lensidx = 0; lensidx < num_lenses; lensidx++)
	{
		size_t num_segments = Segments[lensidx].size();
		num_segments_total += num_segments;
		ids[lensidx].resize(num_segments);
		corners[lensidx].resize(num_segments);
	}
	
	parallel_for_(Range(0, num_segments_total), 
	[&InData, &Segments, &corners, &ids, Detector, num_lenses]
	(Range InRange)
	{
		//Range InRange(0, numpois);
		for (int rangeidx = InRange.start; rangeidx < InRange.end; rangeidx++)
		{
			size_t lensidx=0, poiidx=rangeidx;
			for (lensidx = 0; lensidx < num_lenses; lensidx++)
			{
				if (poiidx >= Segments[lensidx].size())
				{
					poiidx-= Segments[lensidx].size();
				}
				else
				{
					break;
				}
			}
			
			Rect thispoirect = Segments[lensidx][poiidx];
			auto &cornerslocal = corners[lensidx][poiidx];
			auto &idslocal = ids[lensidx][poiidx];
			Point2f offset = thispoirect.tl();

			Detector->detectMarkers(InData.Image(InData.lenses[lensidx].ROI)(thispoirect), cornerslocal, idslocal);
			for (auto &rect : cornerslocal)
			{
				for (Point2f &point : rect)
				{
					point = point + offset;
				}
			}
		}
	});
	
	int NumDetectionsTotal = 0;

	for (size_t lensidx = 0; lensidx < InData.lenses.size(); lensidx++)
	{

		size_t NumSegments = Segments[lensidx].size();
		LensFeatureData &lensDetections = OutData->Lenses[lensidx];
		size_t NumDetectionsBefore = lensDetections.ArucoCorners.size();
		size_t NumDetectionsThis = 0;
		for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
		{
			NumDetectionsThis += ids[lensidx][poiidx].size();
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
			means[i] = ComputeMean(lensDetections.ArucoCorners[i]);
		}
		const float SameWindowSize = 4;
		const Rect2f SameThreshold(-SameWindowSize/2,-SameWindowSize/2,SameWindowSize,SameWindowSize);
		lensDetections.ArucoCorners.reserve(MaxDetectionsAfter);
		lensDetections.ArucoIndices.reserve(MaxDetectionsAfter);
		lensDetections.ArucoCornersReprojected.reserve(MaxDetectionsAfter);
		for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
		{
			
			size_t numdetslocal = ids[lensidx][poiidx].size();
			if (numdetslocal ==0)
			{
				continue;
			}
			vector<ArucoCornerArray> &CornersLocal = corners[lensidx][poiidx];
			vector<int> &IDsLocal = ids[lensidx][poiidx];
			for (size_t PotentialIdx = 0; PotentialIdx < numdetslocal; PotentialIdx++)
			{
				bool found = false;
				Point2f mean = ComputeMean(CornersLocal[PotentialIdx]);
				for (size_t PresentIdx = 0; PresentIdx < lensDetections.ArucoCorners.size(); PresentIdx++)
				{
					if (lensDetections.ArucoIndices[PresentIdx] != IDsLocal[PotentialIdx])
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
						for (size_t corneridx = 0; corneridx < lensDetections.ArucoCorners[PresentIdx].size(); corneridx++)
						{
							//mean the corner locations
							lensDetections.ArucoCorners[PresentIdx][corneridx] = (lensDetections.ArucoCorners[PresentIdx][corneridx]*accumulations[PresentIdx] + CornersLocal[PotentialIdx][corneridx]) / (accumulations[PresentIdx]+1);
						}
						accumulations[PresentIdx]++;
						//cout << "Merging aruco at " << mean << endl;
						break;
					}
				}
				if (found)
				{
					continue;
				}
				means.push_back(mean);
				lensDetections.ArucoIndices.push_back(IDsLocal[PotentialIdx]);
				lensDetections.ArucoCorners.push_back(CornersLocal[PotentialIdx]);
				lensDetections.StereoReprojected.push_back(false);
				accumulations.push_back(1);
			}
		}
		lensDetections.ArucoCornersReprojected.resize(lensDetections.ArucoIndices.size());
		
		NumDetectionsTotal += NumDetectionsThis;
	}
	for (size_t lensidx = 0; lensidx < num_lenses; lensidx++)
	{
		for (size_t segmentidx = 0; segmentidx < Segments[lensidx].size(); segmentidx++)
		{
			auto SegmentROI = Segments[lensidx][segmentidx];
			OutData->ArucoSegments.emplace_back(SegmentROI.tl() + InData.lenses[lensidx].ROI.tl(), SegmentROI.size());
		}
	}
	return NumDetectionsTotal;
}

int DetectArucoSegmented(CameraImageData InData, CameraFeatureData *OutData, const vector<Rect> &Segments, aruco::ArucoDetector* Detector)
{
	size_t NumSegments = Segments.size();
	if (NumSegments == 0)
	{
		return 0;
	}
	size_t num_lenses = InData.lenses.size();

	vector<vector<Rect>> POILensed(num_lenses);
	vector<int> NumLensPOI(Segments.size());
	for (size_t lensidx = 0; lensidx < num_lenses; lensidx++)
	{
		const Rect2i &lens_roi = InData.lenses[lensidx].ROI;
		for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
		{
			bool contained = lens_roi.contains(Segments[poiidx].tl()) && lens_roi.contains(Segments[poiidx].br());	
			if (contained)
			{
				Rect segment_moved = Segments[poiidx];
				segment_moved.x -= InData.lenses[lensidx].ROI.x;
				segment_moved.y -= InData.lenses[lensidx].ROI.y;
				POILensed[lensidx].push_back(segment_moved);
				NumLensPOI[poiidx]++;
			}
		}
	}

	for (size_t poiidx = 0; poiidx < NumSegments; poiidx++)
	{
		assert(NumLensPOI[poiidx] == 1);
	}

	return DetectArucoSegmented(InData, OutData, POILensed, Detector);
}

void PolyCameraArucoMerge(CameraFeatureData &InOutData)
{
	size_t numlenses = InOutData.Lenses.size();
	if (numlenses < 2)
	{
		return;
	}
	for (size_t lens1idx = 0; lens1idx < numlenses; lens1idx++)
	{
		auto& lens1data = InOutData.Lenses[lens1idx];
		Vec3d l1p = lens1data.LensTransform.translation(), l1d;
		for (size_t lens2idx = lens1idx+1; lens2idx < numlenses; lens2idx++)
		{
			auto& lens2data = InOutData.Lenses[lens2idx];
			Vec3d l2p = lens2data.LensTransform.translation(), l2d;
			for (size_t aruco1idx = 0; aruco1idx < lens1data.ArucoIndices.size(); aruco1idx++)
			{
				for (size_t aruco2idx = 0; aruco2idx < lens2data.ArucoIndices.size(); aruco2idx++)
				{
					if (lens1data.ArucoIndices[aruco1idx] != lens2data.ArucoIndices[aruco2idx])
					{
						continue;
					}
					//same index, try intersecting
					float error = 0;
					std::vector<Point2f> &aruco1corners = lens1data.ArucoCorners[aruco1idx], aruco1cornersundist;
					std::vector<Point2f> &aruco2corners = lens2data.ArucoCorners[aruco2idx], aruco2cornersundist;
					assert(aruco1corners.size() == aruco2corners.size());
					undistortPoints(aruco1corners, aruco1cornersundist, lens1data.CameraMatrix, lens1data.DistanceCoefficients);
					undistortPoints(aruco2corners, aruco2cornersundist, lens2data.CameraMatrix, lens2data.DistanceCoefficients);
					std::vector<Point3d> intersections(aruco1corners.size());
					for (size_t corneridx = 0; corneridx < aruco1corners.size(); corneridx++)
					{
						//warn : it may be the opposite
						l1d = lens1data.LensTransform.rotation() * Vec3d(aruco1cornersundist[corneridx].x, aruco1cornersundist[corneridx].y, 1);
						l2d = lens2data.LensTransform.rotation() * Vec3d(aruco2cornersundist[corneridx].x, aruco2cornersundist[corneridx].y, 1);
						Vec3d p1, p2;
						bool intersected = ClosestPointsOnTwoLine(l1p, l1d, l2p, l2d, p1, p2);
						if (!intersected)
						{
							error = +INFINITY;
							break;
						}
						error += sqrt(GetVectorLengthSquared(p1-p2));
						intersections[corneridx] = (p1+p2)/2;
					}
					if (error/aruco1corners.size() < 0.01)
					{
						InOutData.ArucoCornersStereo.push_back(intersections);
						InOutData.ArucoIndicesStereo.push_back(lens1data.ArucoIndices[aruco1idx]);
						lens1data.StereoReprojected[aruco1idx] = true;
						lens2data.StereoReprojected[aruco2idx] = true;
					}
				}
			}
		}
	}
}

int DetectArucoSegmented(CameraImageData InData, CameraFeatureData *OutData, int MaxArucoSize, Size Segments)
{
	assert(OutData != nullptr);
	MakeDetectors();

	vector<vector<Rect>> ROIs;
	ROIs.resize(InData.lenses.size());
	for (size_t lensidx = 0; lensidx < InData.lenses.size(); lensidx++)
	{
		Rect2i framesize = InData.lenses[lensidx].ROI;
		ROIs[lensidx].reserve(Segments.area());
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
				ROIs[lensidx].emplace_back(Point2i(xstart, ystart), Size2i(xend - xstart, yend - ystart));
			}
		}
	}
	

	
	return DetectArucoSegmented(InData, OutData, ROIs, GlobalDetector.get());
}

int DetectAruco(CameraImageData InData, CameraFeatureData *OutData)
{
	assert(OutData != nullptr);
	assert(InData.lenses.size() == 1);
	//TODO : support stereo
	MakeDetectors();

	Size framesize = InData.Image.size();
	Size rescaled = Size2f(framesize)*GetReductionFactor();
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


	vector<ArucoCornerArray> &corners = OutData->Lenses[0].ArucoCorners;
	vector<int> &IDs = OutData->Lenses[0].ArucoIndices;

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
	OutData->Lenses[0].ArucoCornersReprojected.resize(corners.size(), {});
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

int DetectArucoPOI(CameraImageData InData, CameraFeatureData *OutData, const vector<vector<Point3d>> &POIs)
{
	assert(OutData != nullptr);
	MakeDetectors();
	Size framesize = InData.Image.size();
	vector<Rect> poirects = GetPOIRects(POIs, framesize, OutData->CameraTransform, InData.lenses[0].CameraMatrix, InData.lenses[0].distanceCoeffs); //TODO : Support stereo

	return DetectArucoSegmented(InData, OutData, poirects, POIDetector.get());
}