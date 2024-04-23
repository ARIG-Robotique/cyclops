#include "ArucoPipeline/StaticObject.hpp"

#include <opencv2/calib3d.hpp>

#include <Misc/math3d.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <Cameras/Camera.hpp>

using namespace cv;
using namespace std;

StaticObject::StaticObject(bool InRelative, String InName)
{
	Unique = true;
	CoplanarTags = true;
	Relative = InRelative;
	Name = InName;
	const double yamp = 0.5, xamp = 0.75;
	double size = 0.1;
	//vector<int> numbers = {20, 22, 21, 23};
	vector<int> numbers = {22, 23, 20, 21};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromZX(Vec3d(0,0,1), Vec3d(1,0,0));
		Vec3d pos = Vec3d(i%2 ? xamp : -xamp, i>=2 ? yamp : -yamp, 0.0);
		Affine3d markertransform = Affine3d(markerrot, pos);
		ArucoMarker marker(size, numbers[i], markertransform);
		if (marker.number >= 0)
		{
			markers.push_back(marker);
		}
	}
}

StaticObject::~StaticObject()
{
}

bool StaticObject::SetLocation(Affine3d InLocation, uint64_t Tick)
{
	if (Relative)
	{
		return TrackedObject::SetLocation(InLocation, Tick);
	}
	
	Location = Affine3d::Identity();
	return false;
}

bool StaticObject::ShouldBeDisplayed(uint64_t Tick) const
{
	if (Relative)
	{
		return TrackedObject::ShouldBeDisplayed(Tick);
	}
	else
	{
		return true;
	}
}

vector<ObjectData> StaticObject::ToObjectData() const
{
	ObjectData packet;
	packet.type = Relative ? ObjectType::ReferenceRelative : ObjectType::ReferenceAbsolute;
	packet.name = Name;
	packet.location = Location;
	packet.Childs = GetMarkersAndChilds();
	return {packet};
}