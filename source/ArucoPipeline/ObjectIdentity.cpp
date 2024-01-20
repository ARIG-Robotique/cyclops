#include "ArucoPipeline/ObjectIdentity.hpp"
#include <math3d.hpp>
#include <metadata.hpp>
#include <Visualisation/BoardGL.hpp>
#include <cassert>
#include <map>
using namespace std;

std::optional<GLObject> ObjectData::ToGLObject() const
{
	static const map<ObjectType, MeshNames> PacketToMesh = 
	{
		{ObjectType::Robot, MeshNames::robot},
		{ObjectType::Camera, MeshNames::brio},
		{ObjectType::ReferenceAbsolute, MeshNames::arena},
		{ObjectType::ReferenceRelative, MeshNames::arena},
		{ObjectType::Tag, MeshNames::tag},
		{ObjectType::TopTracker, MeshNames::toptracker},
		{ObjectType::TeamTracker, MeshNames::trackercube}
	};

	auto foundmesh = PacketToMesh.find(type);
	if (foundmesh == PacketToMesh.end())
	{
		return nullopt;
	}

	GLObject obj;
	obj.type = foundmesh->second;
	obj.location = Affine3DToGLM(location);
	obj.metadata = metadata;
	return obj;
}

vector<GLObject> ObjectData::ToGLObjects(const vector<ObjectData>& data)
{
	vector<GLObject> outobj;
	outobj.reserve(data.size());
	for (size_t i = 0; i < data.size(); i++)
	{
		auto obj = data[i].ToGLObject();
		if (!obj.has_value())
		{
			continue;
		}
		outobj.push_back(obj.value());
		vector<GLObject> childs = ObjectData::ToGLObjects(data[i].Childs);
		for (size_t i = 0; i < childs.size(); i++)
		{
			childs[i].location = obj.value().location * childs[i].location; //apply parent transform to child
		}
		outobj.insert(outobj.end(), childs.begin(), childs.end());
	}
	return outobj;
}