

#include <ArucoPipeline/SolarPanel.hpp>

#include <opencv2/calib3d.hpp>
#include <Misc/math3d.hpp>

using namespace std;
using namespace cv;

SolarPanel::SolarPanel()
	:TrackedObject()
{
	Affine3d offset = Affine3d::Identity().translate(Vec3d(0,-15/1000.0,0));
	markers = {
		ArucoMarker(37.5/1000.0, 47, offset)
	};
	Unique=true;
	Name="Solar Panels";

	for (size_t i = 0; i < PanelPositions.size(); i++)
	{
		PanelPositions[i] = GetPanelPosition(i);
	}
}

Affine3d SolarPanel::GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
	map<int, ArucoCornerArray> &ReprojectedCorners)
{

	std::vector<ArucoViewCameraLocal> SeenMarkers;
	Surface = GetSeenMarkers(CameraData, SeenMarkers);
	const auto& markerobj = markers[0];
	auto &flatobj = markerobj.GetObjectPointsNoOffset();
	ReprojectionError = 0;
	Affine3d WorldToCam = CameraData.CameraTransform.inv();
	vector<Point2d> ImageSolarPanels;
	cv::projectPoints(PanelPositions, WorldToCam.rvec(), WorldToCam.translation(), CameraData.CameraMatrix, CameraData.DistanceCoefficients, ImageSolarPanels);
	auto UpVector = GetAxis(WorldToCam.rotation(), 2);
	PanelSeenLastTick.fill(false);
	for (auto &marker : SeenMarkers)
	{
		auto& flatimg = marker.CameraCornerPositions;

		Point2d AimPos = (flatimg[0]+flatimg[1])/2.0;
		int closest = -1;
		double smallestDistance = INFINITY;
		for (size_t i = 0; i < ImageSolarPanels.size(); i++)
		{
			auto &PanelLocation = ImageSolarPanels[i];
			auto deltapos = PanelLocation-AimPos;
			double deltasq = deltapos.dot(deltapos);
			if (deltasq < smallestDistance)
			{
				closest = (int)i;
				smallestDistance = deltasq;
			}
		}
		smallestDistance = sqrt(smallestDistance);
		double LocationTolerance = arcLength(flatimg, true); //in pixels
		
		if (smallestDistance>LocationTolerance)
		{
			//cout << "Distance to closest (" << closest << ") too big " << smallestDistance << " (closest panel is at " << ImageSolarPanels[closest] << ", seen is at " << AimPos << ") rejected." << endl;
			continue;
		}
		
		Mat rvec = Mat::zeros(3, 1, CV_64F), tvec = Mat::zeros(3, 1, CV_64F);
		bool solved = false;
		try
		{
			solved = SolvePnPUpright(UpVector, 0.8, flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvec, tvec, false, SOLVEPNP_IPPE_SQUARE);
			//solvePnPGeneric(flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvecs, tvecs, false, SOLVEPNP_IPPE_SQUARE);
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			continue;
		}
		if (!solved)
		{
			continue;
		}
		
		
		solvePnPRefineLM(flatobj, flatimg, CameraData.CameraMatrix, CameraData.DistanceCoefficients, rvec, tvec);

		Matx33d rotationMatrix; //Matrice de rotation Camera -> Tag
		Rodrigues(rvec, rotationMatrix);
		Matx33d RefinedRotation = CameraData.CameraTransform.rotation() * rotationMatrix;
		PanelSeenLastTick[closest] = true;
		auto & rot = PanelRotations[closest];
		rot = GetRotZ(RefinedRotation);
		//cout << "Panel " << closest << " has a rotation of " << PanelRotations[closest]*180.0/M_PI << " deg" << endl;
		Affine3d ExactTransform = CameraData.CameraTransform.inv() * Affine3d(MakeRotationFromZX(Vec3d(0,0,1), Vec3d(cos(rot),sin(rot),0)), PanelPositions[closest]) * markerobj.Pose; //camera to world to panel to marker
		array<Point2d, ARUCO_CORNERS_PER_TAG> ReprojectedCornersDouble;
		projectPoints(markerobj.GetObjectPointsNoOffset(), ExactTransform.rvec(), ExactTransform.translation(), CameraData.CameraMatrix, CameraData.DistanceCoefficients, ReprojectedCornersDouble);
		auto &ReprojectedCornersStorage = ReprojectedCorners[marker.IndexInCameraData];
		ReprojectedCornersStorage.resize(ReprojectedCornersDouble.size());
		for (size_t i = 0; i < ReprojectedCornersDouble.size(); i++)
		{
			ReprojectedCornersStorage[i] = ReprojectedCornersDouble[i];
		}
	}
	return Affine3d::Identity();
}

bool SolarPanel::SetLocation(cv::Affine3d InLocation, TimePoint Tick)
{
	(void) InLocation;
	for (size_t i = 0; i < PanelLastSeenTime.size(); i++)
	{
		if (PanelSeenLastTick[i])
		{
			PanelLastSeenTime[i] = Tick;
		}
	}
	return true;
}

vector<ObjectData> SolarPanel::ToObjectData() const
{
	vector<ObjectData> objects;
	for (size_t i = 0; i < PanelPositions.size(); i++)
	{
		double rot = PanelRotations[i];
		Affine3d location(MakeRotationFromZX(Vec3d(0,0,1), Vec3d(cos(rot),sin(rot),0)), PanelPositions[i]);
		objects.emplace_back(ObjectType::SolarPanel, "Solar Panel " + to_string(i), location, PanelLastSeenTime[i]);
	}
	return objects;
}

vector<vector<Point3d>> SolarPanel::GetPointsOfInterest() const
{
	static vector<vector<Point3d>> points;
	if (points.size() != 0)
	{
		return points;
	}
	
	points.resize(PanelPositions.size());
	for (size_t i = 0; i < PanelPositions.size(); i++)
	{
		vector<Point3d> &thispanelpoints = points[i];
		thispanelpoints.resize(4);
		auto &thispanel = PanelPositions[i];
		const double offset = 0.1;
		for (int j = 0; j < 4; j++)
		{
			thispanelpoints[j].x = thispanel.x + offset*(j&1 ? 1 : -1);
			thispanelpoints[j].y = thispanel.y + offset*(j>>1 ? 1 : -1);
			thispanelpoints[j].z = thispanel.z;
		}
	}
	return points;
}