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

Affine3d TopTracker::GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
	map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners)
{

	std::vector<ArucoViewCameraLocal> Markers2D, Markers3D;
	float Volume = GetSeenMarkers3D(CameraData, Markers3D);
	Surface = GetSeenMarkers2D(CameraData, Markers2D);
	Affine3d CameraToMarker;

	ArucoMarker markerobj;
	ArucoViewCameraLocal SeenMarker;
	if (Markers3D.size() == 0)
	{
		if (Markers2D.size() == 0)
		{
			return Affine3d::Identity();
		}
		
		ReprojectionError = 0;
		Affine3d WorldToCam = CameraData.WorldToCamera.inv();
		auto UpVector = GetAxis(WorldToCam.rotation(), 2);
		SeenMarker = Markers2D[0];
		auto& flatimg = SeenMarker.CameraCornerPositions;
		markerobj = markers[0];
		auto &flatobj = markerobj.GetObjectPointsNoOffset();
		
		if (Markers2D.size() > 1)
		{
			cerr << "Warning : more than 1 top tracker with tag " << markerobj.number << endl;
		}

		Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
		bool solved = false;
		try
		{
			solved = SolvePnPUpright(UpVector, 0.8, flatobj, flatimg, 
				CameraData.Lenses[SeenMarker.LensIndex].CameraMatrix, CameraData.Lenses[SeenMarker.LensIndex].DistanceCoefficients, 
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
			CameraData.Lenses[SeenMarker.LensIndex].CameraMatrix, CameraData.Lenses[SeenMarker.LensIndex].DistanceCoefficients, 
			rvec, tvec);

		CameraToMarker = Affine3d(rvec, tvec);
	}
	else
	{
		SeenMarker = Markers3D[0];
		CameraToMarker = SeenMarker.FitPlane();
		Surface = Volume * CameraData.FrameSize.area(); //does not work for multiple cameras with different resolutions, but it kinda works
		assert(markers.size() == 1);
		//TODO : Handle multiple markers
	}
	
	Affine3d WorldTransform = CameraData.WorldToCamera * CameraToMarker;
	if (ExpectedHeight.has_value() ) //2cm tolerance
	{
		//bool withinDeltaHeight = abs(WorldTransform.translation()[2] - ExpectedHeight.value()) < 0.02;
		if (!Robot)
		{
			Vec3d LocationOnPlane = LinePlaneIntersection(CameraData.WorldToCamera.translation(), 
				WorldTransform.translation() - CameraData.WorldToCamera.translation(), 
				Vec3d(0,0, ExpectedHeight.value()), Vec3d(0,0,1));
			WorldTransform.translation(LocationOnPlane);
			CameraToMarker = CameraData.WorldToCamera.inv() * WorldTransform; //Camera, to world, to tag
		}
	}
	
	Affine3d MarkerToObject = SeenMarker.AccumulatedTransform * SeenMarker.Marker->Pose;
	Affine3d CameraToObject = CameraToMarker * MarkerToObject;
	cout << "Top tracker " << Name << " is at " << CameraToMarker.translation() << " in camera (" 
		<< (CameraData.WorldToCamera * CameraToObject).translation() << " in world)" << endl;
	//cout << "Panel " << closest << " has a rotation of " << PanelRotations[closest]*180.0/M_PI << " deg" << endl;
	ReprojectSeenMarkers({SeenMarker}, CameraToMarker, CameraData, ReprojectedCorners);
	
	return CameraToObject;
	
}

vector<ObjectData> TopTracker::ToObjectData() const
{
	ObjectData tracker(Robot ? ObjectType::Robot : ObjectType::Pami, Name, Location, LastSeenTick);
	tracker.Childs = GetMarkersAndChilds();
	return {tracker};
}