#include "ArucoPipeline/TopTracker.hpp"

#include <ArucoPipeline/ObjectIdentity.hpp>

#include <Misc/math3d.hpp>
#include <Misc/metadata.hpp>

using namespace cv;
using namespace std;

TopTracker::TopTracker(int MarkerIdx, double MarkerSize, String InName, double InExpectedHeight)
	:ExpectedHeight(InExpectedHeight)
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
	map<int, ArucoCornerArray> &ReprojectedCorners)
{

	std::vector<ArucoViewCameraLocal> SeenMarkers;
	Surface = GetSeenMarkers(CameraData, SeenMarkers);
	if (SeenMarkers.size() == 0)
	{
		return Affine3d::Identity();
	}
	
	ReprojectionError = 0;
	Affine3d WorldToCam = CameraData.CameraTransform.inv();
	auto UpVector = GetAxis(WorldToCam.rotation(), 2);
	auto marker = SeenMarkers[0];
	auto& flatimg = marker.CameraCornerPositions;
	const auto& markerobj = markers[0];
	auto &flatobj = markerobj.GetObjectPointsNoOffset();
	
	if (SeenMarkers.size() > 1)
	{
		cerr << "Warning : more than 1 top tracker with tag " << markerobj.number << endl;
	}

	Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
	bool solved = false;
	try
	{
		solved = SolvePnPUpright(UpVector, 0.8, flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
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
	
	solvePnPRefineLM(flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvec, tvec);

	Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
	Rodrigues(rvec, rotationMatrix);
	Affine3d localTransform(rotationMatrix, tvec);
	//cout << "Panel " << closest << " has a rotation of " << PanelRotations[closest]*180.0/M_PI << " deg" << endl;
	array<Point2d, ARUCO_CORNERS_PER_TAG> ReprojectedCornersDouble;
	projectPoints(markerobj.GetObjectPointsNoOffset(), rvec, tvec, CameraData.CameraMatrix, CameraData.DistanceCoefficients, ReprojectedCornersDouble);
	auto &ReprojectedCornersStorage = ReprojectedCorners[marker.IndexInCameraData];
	ReprojectedCornersStorage.resize(ReprojectedCornersDouble.size());
	for (size_t i = 0; i < ReprojectedCornersDouble.size(); i++)
	{
		ReprojectedCornersStorage[i] = ReprojectedCornersDouble[i];
	}
	return localTransform;
}

vector<ObjectData> TopTracker::ToObjectData() const
{
	ObjectData tracker;
	tracker.name = Name;
	tracker.type = ObjectType::TopTracker;
	tracker.location = Location;
	//tracker.Childs = GetMarkersAndChilds();
	return {tracker};
}