#include "ArucoPipeline/TopTracker.hpp"

#include <ArucoPipeline/ObjectIdentity.hpp>

#include <Misc/math3d.hpp>

#include <iostream>

using namespace cv;
using namespace std;

TopTracker::TopTracker(int MarkerIdx, double MarkerSize, String InName, optional<double> InExpectedHeight, bool InRobot)
	:ExpectedHeight(InExpectedHeight), Robot(InRobot)
{
	Unique = false;
	Name = InName;
	markers.clear();
	Affine3d markertransform = Affine3d::Identity();
	ArucoMarker marker(MarkerSize, MarkerIdx, markertransform);
	markers.push_back(marker);
}

TopTracker::~TopTracker()
{
}

Affine3d TopTracker::GetObjectTransform(const LensFeatureData& LensData, float& Surface, int LensIndex)
{

	std::vector<ArucoViewCameraLocal> Markers2D;
	Surface = GetSeenMarkers2D(LensData, Markers2D, Affine3d(), false, LensIndex);

	if (Markers2D.size() == 0)
	{
		return Affine3d::Identity();
	}

	
	Affine3d WorldToCam = LensData.WorldToLens.inv();
	auto UpVector = GetAxis(WorldToCam.rotation(), 2);
	ArucoViewCameraLocal SeenMarker = Markers2D[0];
	auto& flatimg = SeenMarker.CameraCornerPositions;
	ArucoMarker markerobj = markers[0];
	auto &flatobj = markerobj.GetObjectPointsNoOffset();
	
	if (SeenMarker.LensIndex != LensIndex)
	{
		return Affine3d();
	}
	

	if (Markers2D.size() > 1)
	{
		cerr << "Warning : more than 1 top tracker with tag " << markerobj.number << endl;
	}

	Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
	bool solved = false;
	try
	{
		solved = SolvePnPUpright(UpVector, 0.8, flatobj, flatimg, 
			LensData.CameraMatrix, LensData.DistanceCoefficients, 
			rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return Affine3d::Identity();
	}
	if (!solved)
	{
		return Affine3d::Identity();
	}
	
	solvePnPRefineLM(flatobj, flatimg, 
		LensData.CameraMatrix, LensData.DistanceCoefficients, 
		rvec, tvec);

	Affine3d LensToMarker(rvec, tvec);

	
	Affine3d WorldTransform = LensData.WorldToLens * LensToMarker;
	if (ExpectedHeight.has_value() ) //2cm tolerance
	{
		//bool withinDeltaHeight = abs(WorldTransform.translation()[2] - ExpectedHeight.value()) < 0.02;
		if (!Robot)
		{
			Vec3d LocationOnPlane = LinePlaneIntersection(LensData.WorldToLens.translation(), 
				WorldTransform.translation() - LensData.WorldToLens.translation(), 
				Vec3d(0,0, ExpectedHeight.value()), Vec3d(0,0,1));
			WorldTransform.translation(LocationOnPlane);
			LensToMarker = LensData.WorldToLens.inv() * WorldTransform; //Camera, to world, to tag
		}
	}
	
	Affine3d MarkerToObject = SeenMarker.AccumulatedTransform * SeenMarker.Marker->Pose;
	Affine3d CameraToObject = LensToMarker * MarkerToObject;
	//cout << "Top tracker " << Name << " is at " << LensToMarker.translation() << " in camera (" 
	//	<< (CameraData.WorldToCamera * CameraToObject).translation() << " in world)" << endl;
	//cout << "Panel " << closest << " has a rotation of " << PanelRotations[closest]*180.0/M_PI << " deg" << endl;
	
	return CameraToObject;
	
}

vector<ObjectData> TopTracker::ToObjectData() const
{
	ObjectData tracker(Robot ? ObjectType::Robot : ObjectType::Pami, Name, Location, LastSeenTick);
	tracker.Childs = GetMarkersAndChilds();
	return {tracker};
}