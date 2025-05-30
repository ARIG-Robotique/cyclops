#include "EntryPoints/Mapping.hpp"

#include <string>
#include <iostream>
#include <thread>
#include <filesystem>
#include <vector>
#include <map>


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <Misc/math3d.hpp>
#include <Cameras/Calibfile.hpp>
#include <Misc/GlobalConf.hpp>
#include <ArucoPipeline/TrackedObject.hpp>
#include <Visualisation/BoardGL.hpp>

using namespace std;
using namespace cv;

struct ObservedTag
{
	vector<ArucoCornerArray> Observations;
	vector<Affine3d> CameraPositions;
	vector<double> CameraScores;
	int hierarchy;
	int ID;
};

struct SolvableView
{
	vector<int> Unseen;
	vector<int> Seen;
};


const string MappingFolderName = "SFM/";


SolvableView GetUnseenSolvable(const CameraFeatureData& ImageIDs, const TrackedObject* SolvedTags)
{
	#if 1
	(void) ImageIDs;
	(void) SolvedTags;
	return SolvableView();
	#else
	SolvableView view;
	for (size_t i = 0; i < ImageIDs.ArucoIndices.size(); i++)
	{
		bool AlreadySeen = false;
		for (size_t j = 0; j < SolvedTags->markers.size(); j++)
		{
			if (SolvedTags->markers[j].number == ImageIDs.ArucoIndices[i])
			{
				AlreadySeen = true;
				break;
			}
		}
		if (AlreadySeen)
		{
			view.Seen.push_back(i);
		}
		else
		{
			view.Unseen.push_back(i);
		}
		
	}
	return view;
	#endif
}


void MappingSolve(void)
{
	#if 1
	assert(0);
	#else
	Mat CameraMatrix, DistortionCoefficients;
	Size CameraResolution;
	if (!readCameraParameters(MappingFolderName + "calibration", CameraMatrix, DistortionCoefficients, CameraResolution))
	{
		cerr << "Failed to read calibration, could not init mapping" <<endl;
		return;
	}
	vector<Mat> images;
	vector<string> imagenames;
	//load images
	for (const auto &i : filesystem::directory_iterator(MappingFolderName))
	{
		if (i.is_regular_file() && i.path().extension() != ".yaml")
		{
			Mat image;
			try
			{

				image = imread(i.path());
				cout << "Found image at " << i.path() << endl;
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				continue;
			}
			images.push_back(image);
			imagenames.push_back(i.path().filename());
		}
	}
	size_t numim = images.size();
	cout << "Found " << numim << " images for SFM" <<endl;
	if (numim == 0)
	{
		return;
	}
	//detect tags
	vector<CameraFeatureData> ImageData;
	map<int, float> ArucoSizes;
	//ArucoSizes[55] = 0.075;
	//ArucoSizes[60] = 0.1;

	ImageData.resize(numim); 
	auto& detector = GetArucoDetector();

	parallel_for_(Range(0, images.size()), [&](const Range& range)
	{
		for(int i = range.start; i < range.end; i++)
		{
			Mat imageundist;
			undistort(images[i], imageundist, CameraMatrix, DistortionCoefficients);
			CameraFeatureData &ThisData = ImageData[i];
			ThisData.CameraMatrix = CameraMatrix;
			ThisData.DistanceCoefficients = Mat::zeros(4,1, CV_64F);
			ThisData.WorldToCamera = Affine3d::Identity();
			detector.detectMarkers(imageundist, ThisData.ArucoCorners, ThisData.ArucoIndices);
			cout << "Image " << imagenames[i] << " has " << ThisData.ArucoIndices.size() << " detected tags" << endl;
		}
	});

	vector<ObjectData> vizdata;
	
	TrackedObject SolvedTagsObject;
	{
		cout << "What ID is the reference ? (integer)" <<endl;
		int refid; float refsize;
		cin >> refid;
		cout << "What is the side length of it ? (float, meters)" <<endl;
		cin >> refsize;
		ArucoMarker referenceMarker(refsize, refid);
		SolvedTagsObject.markers.push_back(referenceMarker);
		ObjectData d;
		d.type = ObjectType::Tag;
		d.name = referenceMarker.number;
		d.metadata["sideLength"] = referenceMarker.sideLength;
		d.metadata["number"] = referenceMarker.number;
		d.location = referenceMarker.Pose;
		vizdata.push_back(d);
	}

	int i = 1;
	//do the actual solve
	while (i < 255)
	{
		vector<ObservedTag> Observations;
		cout << "Hierarchy level " << i <<endl;
		//detect solvable cameras/images
		for (size_t j = 0; j < numim; j++)
		{
			CameraFeatureData& image = ImageData[j];
			SolvableView view = GetUnseenSolvable(image, &SolvedTagsObject);
			if (view.Seen.size() == 0 || view.Unseen.size() ==0)
			{
				continue;
			}
			float Surface, Error;
			map<int, ArucoCornerArray> ReprojectedCorners;
			Affine3d CameraToObject = SolvedTagsObject.GetObjectTransform(image, Surface, Error, ReprojectedCorners);
			Affine3d CameraPos = CameraToObject.inv();
			{
				//add camera to visualizer
				ObjectData camdata;
				camdata.type = ObjectType::Camera;
				camdata.name = j;
				camdata.location = CameraPos;
				vizdata.push_back(camdata);
			}
			for (size_t k = 0; k < view.Unseen.size(); k++)
			{
				int tagarrayindex = view.Unseen[k];
				int tagid = image.ArucoIndices[tagarrayindex];
				int obsindex = -1;
				for (size_t l = 0; l < Observations.size(); l++)
				{
					if (Observations[l].ID == tagid)
					{
						obsindex = l;
						break;
					}
				}
				if (obsindex == -1)
				{
					ObservedTag t;
					t.hierarchy = i;
					t.ID = tagid;
					obsindex = Observations.size();
					Observations.push_back(t);
				}
				ObservedTag &observed = Observations[obsindex];
				observed.CameraPositions.push_back(CameraPos);
				observed.Observations.push_back(image.ArucoCorners[tagarrayindex]);
				observed.CameraScores.push_back(Surface/(Error+0.1));
			}
		}
		if (Observations.size() == 0)
		{
			cout << "Stopped at hierarchy level " << i << " (no more solves possible)" << endl;
			break;
		}
		//solve tags
		for (size_t j = 0; j < Observations.size(); j++)
		{
			ObservedTag &observed = Observations[j];
			int numviews = observed.CameraPositions.size();
			cout << "\tTag " << observed.ID << " is seen in " << numviews << " image(s)" << endl;
			auto foundsize = ArucoSizes.find(observed.ID);
			if (foundsize == ArucoSizes.end())
			{
				float inputsize;
				cout << "What is the side length of tag ID " << observed.ID << " ? (float, meter)" << endl;
				cin >> inputsize;
				ArucoSizes[observed.ID] = inputsize;
				foundsize = ArucoSizes.find(observed.ID);
			}
			if (foundsize == ArucoSizes.end())
			{
				cout << "Cannot solve tag location for tag ID " << observed.ID << " as ID is not in the size map" <<endl;
				continue;
			}
			ArucoMarker marker(foundsize->second, foundsize->first);
			Affine3d &TagLocation = marker.Pose;
			if (numviews == 1)
			{
				TrackedObject MObj;
				MObj.markers.push_back(marker);
				CameraFeatureData data;
				data.CameraMatrix = CameraMatrix;
				data.DistanceCoefficients = Mat::zeros(4,1, CV_64F);
				data.ArucoCorners = observed.Observations;
				data.ArucoIndices = {observed.ID};
				data.WorldToCamera = observed.CameraPositions[0];
				float surface, error;
				map<int, ArucoCornerArray> ReprojectedCorners;
				//roundabout way of getting a solvepnp, but at least i'm using stuff that's already made
				TagLocation = MObj.GetObjectTransform(data, surface, error, ReprojectedCorners);
				cout << "\tTag " << observed.ID << " was solved with solvePNP : surface=" << surface << "px² error=" << error << "px/pt" <<endl;
			}
			else
			{
				//triangulate corners, then find the best location
				using tript = Point3d;
				struct SortableView
				{
					float Score;
					Affine3d CameraPos;
					ArucoCornerArray corners;
					vector<Vec3d> lines;

					bool operator<(SortableView& other) {return Score < other.Score;};

					void ComputeScore(double CameraScore)
					{
						double area = contourArea(corners, false);
						Score = area*CameraScore;
					}
					void ComputeLines(Mat CameraMatrix)
					{
						lines.resize(corners.size());
						Mat invmat = CameraMatrix.inv();
						for (size_t i = 0; i < corners.size(); i++)
						{
							Point3d pt(corners[i].x, corners[i].y, 1.0);
							Mat pt3d = CameraPos.rotation() * (invmat * Vec3d(pt));
							lines[i] = pt3d;
							//cout << lines[i] <<endl;
							lines[i] = NormaliseVector(lines[i]);
						}
						
					}
				};
				int numpt = observed.Observations[0].size();

				vector<SortableView> sorted(numviews);
				for (int k = 0; k < numviews; k++)
				{
					sorted[k].CameraPos = observed.CameraPositions[k];
					assert(numpt == (int)observed.Observations[k].size());
					sorted[k].corners = ArucoCornerArray(observed.Observations[k]);
					sorted[k].ComputeScore(observed.CameraScores[k]);
					sorted[k].ComputeLines(CameraMatrix);
				}

				sort(sorted.begin(), sorted.end());
				vector<tript> triangulatedPoints(numpt);
				double AccumulatedArea = sorted[0].Score + sorted[1].Score;
				
				//first rough intersection using the two first
				for (int k = 0; k < numpt; k++)
				{
					Vec3d l0p, l1p;
					ClosestPointsOnTwoLine(sorted[0].CameraPos.translation(), sorted[0].lines[k],
						sorted[1].CameraPos.translation(), sorted[1].lines[k], l0p, l1p);
					triangulatedPoints[k] = (l0p*sorted[0].Score + l1p*sorted[1].Score) / AccumulatedArea;
				}

				for (size_t k = 2; k < sorted.size(); k++)
				{
					for (int l = 0; l < numpt; l++)
					{
						tript proj = ProjectPointOnLine(triangulatedPoints[l], sorted[k].CameraPos.translation(), sorted[k].lines[l]);
						triangulatedPoints[l] = proj;
					}
					AccumulatedArea += sorted[k].Score;
				}

				
				tript mean;
				Point3d Xmean(0), Ymean(0);
				//double xAcc=0, yAcc=0;
				assert(numpt == 4);
				for (int i = 0; i < 4; i++)
				{
					mean += triangulatedPoints[i];
				}
				mean /= 4;
				Xmean += triangulatedPoints[1] - triangulatedPoints[0];
				Xmean += triangulatedPoints[2] - triangulatedPoints[3];
				Ymean += triangulatedPoints[0] - triangulatedPoints[3];
				Ymean += triangulatedPoints[1] - triangulatedPoints[2];
				Vec3d X(Xmean), Y(Ymean);
				cout << "\t\tLength is supposed to be " << marker.sideLength << endl;
				cout << "\t\tLength of X is " << sqrt(GetVectorLengthSquared(X))/2 << endl;
				cout << "\t\tLength of Y is " << sqrt(GetVectorLengthSquared(Y))/2 << endl;
				X = NormaliseVector(X);
				Y = NormaliseVector(Y);
				cout << "\t\tOrthogonality is " << X.ddot(Y) << " (should be close to 0)" << endl;
				auto R = MakeRotationFromXY(X, Y);
				TagLocation = Affine3d(R, mean);
			}
			SolvedTagsObject.markers.push_back(marker);
			ObjectData d;
			d.type = ObjectType::Tag;
			d.name = marker.number;
			d.metadata["sideLength"] = marker.sideLength;
			d.metadata["number"] = marker.number;
			d.location = marker.Pose;
			vizdata.push_back(d);
		}
		
		i++;
	}
	// display what we found
	BoardGL visualiser;
	visualiser.Init();
	visualiser.LoadTags();
	
	while (visualiser.Tick(ObjectData::ToGLObjects(vizdata)))
	{
		
	}

	#endif
}