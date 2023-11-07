#include "ArucoPipeline/TopTracker.hpp"

#include "ArucoPipeline/ObjectIdentity.hpp"

#include "math3d.hpp"
#include "metadata.hpp"

using namespace cv;
using namespace std;

TopTracker::TopTracker(int MarkerIdx, double MarkerSize, String InName)
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

vector<ObjectData> TopTracker::ToObjectData() const
{
	ObjectData robot;
	robot.name = Name;
	robot.type = ObjectType::TopTracker;
	robot.location = Location;
	robot.Childs = GetMarkersAndChilds();
	return {robot};
}