#include "ArucoPipeline/TrackedObject.hpp"
#include <ArucoPipeline/TrackerCube.hpp> // for the test end
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


#include <Misc/math3d.hpp>
#include <Misc/GlobalConf.hpp>
#include <Cameras/Camera.hpp>
#include <Visualisation/BoardGL.hpp>

using namespace cv;
using namespace std;


array<Point3d, 4> ArucoMarker::GetObjectPointsNoOffset(double SideLength)
{
	double sql2 = SideLength*0.5;
	return {
		Point3d(-sql2, sql2, 0.0),
		Point3d(sql2, sql2, 0.0),
		Point3d(sql2, -sql2, 0.0),
		Point3d(-sql2, -sql2, 0.0)
	};
}

const array<Point3d, 4>& ArucoMarker::GetObjectPointsNoOffset() const
{
	if (ObjectPointsNoOffset.size() != 4)
	{
		assert(sideLength > 0);
		ObjectPointsNoOffset = GetObjectPointsNoOffset(sideLength);
	}
	
	assert(ObjectPointsNoOffset.size() == 4);
	return ObjectPointsNoOffset;
}

float TrackedObject::ArucoViewCameraLocal::GetSurface()
{
	return contourArea(CameraCornerPositions, false);
}

float TrackedObject::ArucoViewCameraLocal::GetVolume()
{
	Point3d mean;
	for (size_t i = 0; i < StereoMarkerCorners.size(); i++)
	{
		mean = mean + (StereoMarkerCorners[i]);
	}
	mean = mean / (double)StereoMarkerCorners.size();
	double dev = 0;
	for (size_t i = 0; i < StereoMarkerCorners.size(); i++)
	{
		Point3d diff = mean - StereoMarkerCorners[i];
		dev += sqrt((diff).ddot(diff));
	}
	dev /= StereoMarkerCorners.size() * sqrt(mean.ddot(mean));
	return dev;
}

cv::Affine3d TrackedObject::ArucoViewCameraLocal::FitPlane()
{
	assert(StereoMarkerCorners.size() == 4);
	Point3d xaxis = (StereoMarkerCorners[1] - StereoMarkerCorners[0]) + (StereoMarkerCorners[2] - StereoMarkerCorners[3]);
	Point3d yaxis = (StereoMarkerCorners[3] - StereoMarkerCorners[0]) + (StereoMarkerCorners[2] - StereoMarkerCorners[1]);
	xaxis = NormaliseVector(xaxis);
	yaxis = NormaliseVector(yaxis);
	auto rot = MakeRotationFromXY(xaxis, yaxis);
	Point3d mean;
	for (size_t i = 0; i < StereoMarkerCorners.size(); i++)
	{
		mean = mean + (StereoMarkerCorners[i]);
	}
	mean = mean / (double)StereoMarkerCorners.size();
	return Affine3d(rot, mean);
}


TrackedObject::TrackedObject()
	:Unique(true),
	CoplanarTags(false),
	Location(cv::Affine3d::Identity())
{
	LocationFilter = cv::KalmanFilter(9, 3, 0, CV_64F);

	cv::setIdentity(LocationFilter.processNoiseCov, cv::Scalar::all(1e-5)); //0.1mm error
	cv::setIdentity(LocationFilter.measurementNoiseCov, cv::Scalar::all(1e-1));//1mm error
	cv::setIdentity(LocationFilter.errorCovPost, cv::Scalar::all(1));
};

bool TrackedObject::SetLocation(Affine3d InLocation, TimePoint Tick)
{
	Location = InLocation;
	double dt = chrono::duration<double>(Tick - LastSeenTick).count();
	LastSeenTick = Tick;
	if (1) //disable kalman filtering
	{
		return true;
	}
	
	
	dt /= getTickFrequency();
	LocationFilter.transitionMatrix = Mat::eye(9,9, CV_64F);
	double v = dt*10;//filter 10Hz
	double a = v*v/2;
	for (int i = 0; i < 6; i++)
	{
		LocationFilter.transitionMatrix.at<double>(i, 3+i) = v;
	}
	for (int i = 0; i < 3; i++)
	{
		LocationFilter.transitionMatrix.at<double>(i, 6+i) = a;
		LocationFilter.measurementMatrix.at<double>(i, i) = 1;
		LocationFilter.measurementMatrix.at<double>(i, 3+i) = v;
		LocationFilter.measurementMatrix.at<double>(i, 6+i) = a;
	}
	//cout << "Transition matrix: " << endl << LocationFilter.transitionMatrix << endl;
	//cout << "Measurement matrix: " << endl << LocationFilter.measurementMatrix << endl;
	Mat locmat(InLocation.translation());
	//cout << "locmat: " <<endl << locmat << endl;
	LocationFilter.correct(locmat);
	LocationFilter.predict();
	Mat correctmat = LocationFilter.statePost;
	//cout << "corrmat: " << correctmat.t() << endl;
	Vec3d correctloc(correctmat.rowRange(0, 3));

	Location.translation(correctloc);
	return true;
}

bool TrackedObject::ShouldBeDisplayed(TimePoint Tick) const
{
	(void) Tick;
	return LastSeenTick != TimePoint();
}

Affine3d TrackedObject::GetLocation() const
{
	return Location;
}

bool TrackedObject::FindTag(int MarkerID, ArucoMarker& Marker, Affine3d& TransformToMarker)
{
	for (size_t i = 0; i < markers.size(); i++)
	{
		if (markers[i].number == MarkerID)
		{
			Marker = markers[i];
			TransformToMarker = Affine3d::Identity();
			return true;
		}
	}
	for (size_t i = 0; i < childs.size(); i++)
	{
		if(childs[i]->FindTag(MarkerID, Marker, TransformToMarker))
		{
			TransformToMarker = childs[i]->Location * TransformToMarker;
			return true;
		}
	}
	return false;
}

void TrackedObject::GetObjectPoints(vector<vector<Point3d>>& MarkerCorners, vector<int>& MarkerIDs, Affine3d rootTransform, vector<int> filter)
{
	for (size_t i = 0; i < markers.size(); i++)
	{
		ArucoMarker& marker = markers[i];
		//If filter is not empty and the number wasn't found in he filter
		if (filter.size() != 0 && std::find(filter.begin(), filter.end(), marker.number) == filter.end())
		{
			continue;
		}
		auto& cornerslocal = marker.GetObjectPointsNoOffset();
		MarkerIDs.push_back(marker.number);
		vector<Point3d> cornersworld;
		for (size_t i = 0; i < cornerslocal.size(); i++)
		{
			cornersworld.push_back(rootTransform * (marker.Pose * cornerslocal[i]));
		}
		MarkerCorners.push_back(cornersworld);
	}
	for (size_t i = 0; i < childs.size(); i++)
	{
		auto child = childs[i];
		child->GetObjectPoints(MarkerCorners, MarkerIDs, rootTransform * child->Location, filter);
	}
}

float TrackedObject::GetSeenMarkers3D(const CameraFeatureData& CameraData, vector<ArucoViewCameraLocal> &MarkersSeen, cv::Affine3d AccumulatedTransform)
{
	float surface = 0;
	MarkersSeen.reserve(markers.size());
	for (size_t i = 0; i < markers.size(); i++)
	{
		for (size_t j = 0; j < CameraData.ArucoIndicesStereo.size(); j++)
		{
			if (markers[i].number == CameraData.ArucoIndicesStereo[j])
			{
				//gotcha!
				ArucoViewCameraLocal seen;
				seen.Marker = &markers[i];
				seen.LensIndex = -1;
				seen.IndexInCameraData = j;
				seen.StereoMarkerCorners = CameraData.ArucoCornersStereo[j];
				seen.AccumulatedTransform = AccumulatedTransform;
				auto &cornersLocal = markers[i].GetObjectPointsNoOffset();
				Affine3d TransformToObject = AccumulatedTransform * markers[i].Pose;
				seen.LocalMarkerCorners.reserve(cornersLocal.size());
				for (size_t k = 0; k < cornersLocal.size(); k++)
				{
					seen.LocalMarkerCorners.push_back(TransformToObject * cornersLocal[k]);
				}
				MarkersSeen.push_back(seen);
				surface += seen.GetVolume();
			}
		}
	}
	for (size_t i = 0; i < childs.size(); i++)
	{
		auto child = childs[i];
		surface += child->GetSeenMarkers3D(CameraData, MarkersSeen, AccumulatedTransform * child->Location);
	}
	return surface;
}

float TrackedObject::GetSeenMarkers2D(const CameraFeatureData& CameraData, vector<ArucoViewCameraLocal> &MarkersSeen, cv::Affine3d AccumulatedTransform, bool Skip3D)
{
	float surface = 0;
	MarkersSeen.reserve(markers.size());
	for (size_t i = 0; i < markers.size(); i++)
	{
		for (size_t lensidx = 0; lensidx < CameraData.Lenses.size(); lensidx++)
		{
			auto &lensdata = CameraData.Lenses[lensidx];
			for (size_t lensarucoidx = 0; lensarucoidx < lensdata.ArucoIndices.size(); lensarucoidx++)
			{
				if (markers[i].number == lensdata.ArucoIndices[lensarucoidx])
				{
					//gotcha!
					if (lensdata.StereoReprojected[lensarucoidx] && Skip3D)
					{
						continue;
					}
					ArucoViewCameraLocal seen;
					seen.Marker = &markers[i];
					seen.LensIndex = lensidx;
					seen.IndexInCameraData = lensarucoidx;
					seen.CameraCornerPositions = lensdata.ArucoCorners[lensarucoidx];
					seen.AccumulatedTransform = AccumulatedTransform;
					auto &cornersLocal = markers[i].GetObjectPointsNoOffset();
					Affine3d TransformToObject = AccumulatedTransform * markers[i].Pose;
					seen.LocalMarkerCorners.reserve(cornersLocal.size());
					for (size_t k = 0; k < cornersLocal.size(); k++)
					{
						seen.LocalMarkerCorners.push_back(TransformToObject * cornersLocal[k]);
					}
					MarkersSeen.push_back(seen);
					surface += seen.GetSurface();
				}
			}
		}
	}
	for (size_t i = 0; i < childs.size(); i++)
	{
		auto child = childs[i];
		surface += child->GetSeenMarkers2D(CameraData, MarkersSeen, AccumulatedTransform * child->Location);
	}
	return surface;
}

float TrackedObject::ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const Affine3d &CameraToMarker, 
	const CameraFeatureData &CameraData, map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners)
{
	float ReprojectionError = 0;
	for (size_t i = 0; i < MarkersSeen.size(); i++)
	{
		vector<Point2d> cornersreproj;
		auto &seen = MarkersSeen[i];
		if (seen.CameraCornerPositions.size() == 0 || seen.LensIndex == -1)
		{
			continue;
		}
		const auto& lens = CameraData.Lenses[seen.LensIndex];
		Affine3d LensToMarker = lens.CameraToLens.inv() * CameraToMarker;
		projectPoints(seen.LocalMarkerCorners, LensToMarker.rotation(), LensToMarker.translation(), lens.CameraMatrix, lens.DistanceCoefficients, cornersreproj);
		//cout << "reprojecting " << seen.IndexInCameraData << endl;
		auto &reprojectedThisStorage = ReprojectedCorners[{seen.LensIndex, seen.IndexInCameraData}];
		reprojectedThisStorage.resize(cornersreproj.size());
		for (size_t j = 0; j < cornersreproj.size(); j++)
		{
			reprojectedThisStorage[j] = cornersreproj[j];
			Point2f diff = seen.CameraCornerPositions[j] - Point2f(cornersreproj[j]);
			ReprojectionError += sqrt(diff.ddot(diff));
		}
	}
	return ReprojectionError;
}

Affine3d TrackedObject::GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
	map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners)
{
	vector<ArucoViewCameraLocal> SeenStereo, SeenMono;

	float Volume = GetSeenMarkers3D(CameraData, SeenStereo, Affine3d::Identity());
	
	Surface = GetSeenMarkers2D(CameraData, SeenMono, Affine3d::Identity(), markers.size() == 1);
	ReprojectionError = INFINITY;
	
	if (SeenStereo.size() == 0 || SeenMono.size() > SeenStereo.size()+1 || true) //More than one aruco tag seen by both cameras
	{
		int nummarkersseen = SeenMono.size();
		if (nummarkersseen == 0)
		{
			return Affine3d();
		}
		
		int bestlens = 0;
		if (nummarkersseen == 1 || CameraData.Lenses.size() == 1)
		{
			bestlens = SeenMono[0].LensIndex;
		}
		else //find the lens with the most aruco tags detected
		{
			map<int, int> lensViews;
			for (size_t markeridx = 0; markeridx < SeenMono.size(); markeridx++)
			{
				lensViews[SeenMono[markeridx].LensIndex]++;
			}
			int bestlens = lensViews.begin()->first;
			for (auto &&i : lensViews)
			{
				if (i.second > lensViews[bestlens])
				{
					bestlens = i.first;
				}
			}
		}
		
		Affine3d CameraToObject;
		vector<Point3d> flatobj;
		vector<Point2f> flatimg;
		//vector<Point2d> flatreproj;
		flatobj.reserve(nummarkersseen * ARUCO_CORNERS_PER_TAG);
		flatimg.reserve(nummarkersseen * ARUCO_CORNERS_PER_TAG);
		Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
		Affine3d objectToMarker;
		int flags = 0;
		if (nummarkersseen == 1)
		{
			auto& objpts = SeenMono[0].Marker->GetObjectPointsNoOffset();
			flatobj = vector<Point3d>(objpts.begin(), objpts.end());
			SeenMono[0].LocalMarkerCorners = flatobj; //hack to have ReprojectSeenMarkers work wih a single marker too
			flatimg = vector<Point2f>(SeenMono[0].CameraCornerPositions.begin(), SeenMono[0].CameraCornerPositions.end());
			objectToMarker = SeenMono[0].AccumulatedTransform * SeenMono[0].Marker->Pose;
			flags |= SOLVEPNP_IPPE_SQUARE;
		}
		else
		{
			flatobj.reserve(nummarkersseen*ARUCO_CORNERS_PER_TAG);
			flatimg.reserve(nummarkersseen*ARUCO_CORNERS_PER_TAG);
			for (int i = 0; i < nummarkersseen; i++)
			{
				if (SeenMono[i].LensIndex != bestlens)
				{
					continue;
				}
				
				for (int j = 0; j < ARUCO_CORNERS_PER_TAG; j++)
				{
					flatobj.push_back(SeenMono[i].LocalMarkerCorners[j]);
					flatimg.push_back(SeenMono[i].CameraCornerPositions[j]);
				}
			}
			objectToMarker = Affine3d::Identity();
			flags |= CoplanarTags ? SOLVEPNP_IPPE : SOLVEPNP_SQPNP;
		}
		const Mat &distCoeffs = CameraData.Lenses[bestlens].DistanceCoefficients;
		try
		{
			solvePnP(flatobj, flatimg, CameraData.Lenses[bestlens].CameraMatrix, distCoeffs, rvec, tvec, false, flags);
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			return Affine3d::Identity();
		}
		
		solvePnPRefineLM(flatobj, flatimg, CameraData.Lenses[bestlens].CameraMatrix, distCoeffs, rvec, tvec);
		Affine3d LensToMarker = Affine3d(rvec, tvec);
		Affine3d CameraToMarker = CameraData.Lenses[bestlens].CameraToLens * LensToMarker;
		CameraToObject = CameraToMarker * objectToMarker.inv();

		ReprojectionError = ReprojectSeenMarkers(SeenMono, CameraToMarker, CameraData, ReprojectedCorners);
		
		ReprojectionError /= nummarkersseen;
		//cout << "Reprojection error : " << ReprojectionError << endl;

		return CameraToObject;
	}
	else
	{
		if (SeenStereo.size() == 1 || true)
		{
			auto &marker = SeenStereo[0];
			Affine3d LensToMarker = marker.FitPlane();
			Affine3d ObjectToMarker = marker.AccumulatedTransform * marker.Marker->Pose;
			return LensToMarker * ObjectToMarker.inv();
		}
		
		assert(0);
		//TODO
		return Affine3d();
	}
	
}

vector<ObjectData> TrackedObject::GetMarkersAndChilds() const
{
	vector<ObjectData> datas;
	for (auto child : childs)
	{
		vector<ObjectData> thisdata = child->ToObjectData();
		datas.insert(datas.end(), thisdata.begin(), thisdata.end());
	}
	for (size_t i = 0; i < markers.size(); i++)
	{
		const ArucoMarker &m = markers[i];
		ObjectData d;
		d.name = m.number;
		d.type = ObjectType::Tag;
		d.metadata["sideLength"] = m.sideLength;
		d.metadata["number"] = m.number;
		d.location = m.Pose;
		datas.push_back(d);
	}
	return datas;
}

vector<ObjectData> TrackedObject::ToObjectData() const
{
	cerr << "WARNING: Call to base TrackedObject::ToObjectData function. That function is uninplemented. Override it." <<endl;
	return {};
}

vector<vector<Point3d>> TrackedObject::GetPointsOfInterest() const
{
	return {};
}

void TrackedObject::Inspect()
{
	BoardGL board;
	board.Init();
	board.LoadTags();

	vector<ObjectData> datas = ToObjectData();
	while (board.Tick(ObjectData::ToGLObjects(datas)));
}
