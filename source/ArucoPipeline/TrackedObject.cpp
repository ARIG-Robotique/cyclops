#include "ArucoPipeline/TrackedObject.hpp"
#include "ArucoPipeline/TrackerCube.hpp" // for the test env
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


#include "math3d.hpp"
#include "metadata.hpp"
#include "GlobalConf.hpp"
#include "Cameras/Camera.hpp"
#include "Visualisation/BoardGL.hpp"

using namespace cv;
using namespace std;


vector<Point3d> ArucoMarker::GetObjectPointsNoOffset(float SideLength)
{
	float sql2 = SideLength*0.5;
	return {
		Point3d(-sql2, sql2, 0.0),
		Point3d(sql2, sql2, 0.0),
		Point3d(sql2, -sql2, 0.0),
		Point3d(-sql2, -sql2, 0.0)
	};
}

vector<Point3d>& ArucoMarker::GetObjectPointsNoOffset()
{
	return ObjectPointsNoOffset;
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

bool TrackedObject::SetLocation(Affine3d InLocation, unsigned long tick)
{
	double dt = (tick - LastSeenTick);
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

	Location = InLocation;
	Location.translation(correctloc);
	LastSeenTick = tick;
	return true;
}

bool TrackedObject::ShouldBeDisplayed(unsigned long Tick)
{
	return Tick < LastSeenTick + getTickFrequency()*0.1;
}

Affine3d TrackedObject::GetLocation()
{
	return Location;
}

bool TrackedObject::FindTag(int MarkerID, ArucoMarker& Marker, Affine3d& TransformToMarker)
{
	for (int i = 0; i < markers.size(); i++)
	{
		if (markers[i].number == MarkerID)
		{
			Marker = markers[i];
			TransformToMarker = Affine3d::Identity();
			return true;
		}
	}
	for (int i = 0; i < childs.size(); i++)
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
	for (int i = 0; i < markers.size(); i++)
	{
		ArucoMarker& marker = markers[i];
		//If filter is not empty and the number wasn't found in he filter
		if (filter.size() != 0 && std::find(filter.begin(), filter.end(), marker.number) == filter.end())
		{
			continue;
		}
		vector<Point3d> cornerslocal = marker.GetObjectPointsNoOffset();
		MarkerIDs.push_back(marker.number);
		vector<Point3d> cornersworld;
		for (int i = 0; i < cornerslocal.size(); i++)
		{
			cornersworld.push_back(rootTransform * (marker.Pose * cornerslocal[i]));
		}
		MarkerCorners.push_back(cornersworld);
	}
	for (int i = 0; i < childs.size(); i++)
	{
		TrackedObject* child = childs[i];
		child->GetObjectPoints(MarkerCorners, MarkerIDs, rootTransform * child->Location, filter);
	}
}

float TrackedObject::GetSeenMarkers(const CameraFeatureData& CameraData, vector<ArucoViewCameraLocal> &MarkersSeen, cv::Affine3d AccumulatedTransform)
{
	float surface = 0;
	for (int i = 0; i < markers.size(); i++)
	{
		for (int j = 0; j < CameraData.ArucoIndices.size(); j++)
		{
			if (markers[i].number == CameraData.ArucoIndices[j])
			{
				//gotcha!
				ArucoViewCameraLocal seen;
				seen.Marker = &markers[i];
				seen.IndexInCameraData = j;
				seen.CameraCornerPositions = CameraData.ArucoCorners[j];
				seen.AccumulatedTransform = AccumulatedTransform;
				vector<Point3d> &cornersLocal = markers[i].ObjectPointsNoOffset;
				Affine3d TransformToObject = AccumulatedTransform * markers[i].Pose;
				for (int k = 0; k < 4; k++)
				{
					seen.LocalMarkerCorners.push_back(TransformToObject * cornersLocal[k]);
				}
				MarkersSeen.push_back(seen);
				surface += contourArea(CameraData.ArucoCorners[j], false);
			}
			
		}
		
	}
	for (int i = 0; i < childs.size(); i++)
	{
		TrackedObject* child = childs[i];
		surface += child->GetSeenMarkers(CameraData, MarkersSeen, AccumulatedTransform * child->Location);
	}
	return surface;
}

float TrackedObject::ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const Mat &rvec, const Mat &tvec, 
	const CameraFeatureData &CameraData, map<int, vector<Point2f>> &ReprojectedCorners)
{
	float ReprojectionError = 0;
	for (int i = 0; i < MarkersSeen.size(); i++)
	{
		vector<Point2d> cornersreproj;
		projectPoints(MarkersSeen[i].LocalMarkerCorners, rvec, tvec, CameraData.CameraMatrix, CameraData.DistanceCoefficients, cornersreproj);
		ReprojectedCorners[MarkersSeen[i].IndexInCameraData] = vector<Point2f>(cornersreproj.begin(), cornersreproj.end());
		
		for (int j = 0; j < cornersreproj.size(); j++)
		{
			Point2f diff = MarkersSeen[i].CameraCornerPositions[j] - Point2f(cornersreproj[j]);
			ReprojectionError += sqrt(diff.ddot(diff));
		}
	}
	return ReprojectionError;
}

Affine3d TrackedObject::GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
	map<int, vector<Point2f>> &ReprojectedCorners)
{
	vector<ArucoViewCameraLocal> SeenMarkers;
	Surface = GetSeenMarkers(CameraData, SeenMarkers, Affine3d::Identity());
	ReprojectionError = INFINITY;
	int nummarkersseen = SeenMarkers.size();

	if (nummarkersseen <= 0)
	{
		return Affine3d::Identity();
	}
	Affine3d localTransform;
	vector<Point3d> flatobj;
	vector<Point2f> flatimg;
	//vector<Point2d> flatreproj;
	flatobj.reserve(nummarkersseen * 4);
	flatimg.reserve(nummarkersseen * 4);
	Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
	Affine3d objectToMarker;
	int flags = 0;
	if (nummarkersseen == 1)
	{
		flatobj = SeenMarkers[0].Marker->ObjectPointsNoOffset;
		SeenMarkers[0].LocalMarkerCorners = SeenMarkers[0].Marker->ObjectPointsNoOffset; //hack to have ReprojectSeenMarkers work wih a single marker too
		flatimg = SeenMarkers[0].CameraCornerPositions;
		objectToMarker = SeenMarkers[0].AccumulatedTransform * SeenMarkers[0].Marker->Pose;
		flags |= SOLVEPNP_IPPE_SQUARE;
	}
	else
	{
		for (int i = 0; i < nummarkersseen; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				flatobj.push_back(SeenMarkers[i].LocalMarkerCorners[j]);
				flatimg.push_back(SeenMarkers[i].CameraCornerPositions[j]);
			}
		}
		objectToMarker = Affine3d::Identity();
		flags |= CoplanarTags ? SOLVEPNP_IPPE : SOLVEPNP_SQPNP;
	}
	//Mat distCoeffs = Mat::zeros(4,1, CV_64F); //FIXME
	const Mat &distCoeffs = CameraData.DistanceCoefficients;
	solvePnP(flatobj, flatimg, CameraData.CameraMatrix, distCoeffs, rvec, tvec, false, flags);
	solvePnPRefineLM(flatobj, flatimg, CameraData.CameraMatrix, distCoeffs, rvec, tvec);
	ReprojectionError = ReprojectSeenMarkers(SeenMarkers, rvec, tvec, CameraData, ReprojectedCorners);
	
	ReprojectionError /= nummarkersseen;
	//cout << "Reprojection error : " << ReprojectionError << endl;
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	localTransform = Affine3d(rotationMatrix, tvec) * objectToMarker.inv();

	return localTransform;

	
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
		d.metadata = MakeTag(m.sideLength, m.number);
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

void TrackedObject::Inspect()
{
	BoardGL board;
	board.Start();

	board.LoadTags();

	vector<ObjectData> datas = ToObjectData();
	while (board.Tick(ObjectData::ToGLObjects(datas)));
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Mat& CameraMatrix, Mat& DistanceCoefficients)
{
	Mat rvec, tvec;
	//Mat distCoeffs = Mat::zeros(4,1, CV_64F);
	solvePnP(ArucoMarker::GetObjectPointsNoOffset(SideLength), Corners, 
		CameraMatrix, DistanceCoefficients, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	return Affine3d(rotationMatrix, tvec);
}

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Camera* Cam)
{
	Mat rvec, tvec;
	Mat CamMatrix, distCoeffs;
	Cam->GetCameraSettingsAfterUndistortion(CamMatrix, distCoeffs);
	return GetTagTransform(SideLength, Corners, CamMatrix, distCoeffs);
}

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam)
{
	return Tag.Pose * GetTagTransform(Tag.sideLength, Corners, Cam).inv();
}

