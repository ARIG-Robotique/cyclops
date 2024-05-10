#include "ArucoPipeline/ObjectIdentity.hpp"
#include <Misc/math3d.hpp>
#include <Visualisation/BoardGL.hpp>
#include <cassert>
#include <map>
#include <iostream>
using namespace std;

CDFRTeam GetOtherTeam(CDFRTeam InTeam)
{
	switch (InTeam)
	{
	case CDFRTeam::Blue:
		return CDFRTeam::Yellow;
	case CDFRTeam::Yellow:
		return CDFRTeam::Blue;
	default:
		return CDFRTeam::Unknown;
	}
}

std::ostream& operator << (std::ostream& out, CDFRTeam Team)
{
	out << TeamNames.at(Team);
	return out;
}

std::ostream& operator << (std::ostream& out, ObjectType Type)
{
	out << ObjectTypeNames.at(Type);
	return out;
}

std::optional<GLObject> ObjectData::ToGLObject() const
{
	static const map<ObjectType, MeshNames> PacketToMesh = 
	{
		{ObjectType::Camera,			MeshNames::brio},
		{ObjectType::ReferenceAbsolute,	MeshNames::arena},
		{ObjectType::ReferenceRelative,	MeshNames::arena},
		{ObjectType::Tag,				MeshNames::tag},
		{ObjectType::Robot,				MeshNames::toptracker},
		{ObjectType::Pami,				MeshNames::toptracker},
		{ObjectType::SolarPanel,		MeshNames::solarpanel},
		{ObjectType::Fragile,			MeshNames::fragile},
		{ObjectType::Resistant,			MeshNames::resistant},
		{ObjectType::Pot,				MeshNames::pot},
		{ObjectType::PottedPlant,		MeshNames::potted_plant}
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

vector<GLObject> ObjectData::ToGLObjects(const vector<ObjectData>& data, Clock::duration maxAge)
{
	vector<GLObject> outobj;
	outobj.reserve(data.size());
	TimePoint OldCutoff = Clock::now() - maxAge;
	for (size_t i = 0; i < data.size(); i++)
	{
		auto &object = data[i];
		if (object.LastSeen < OldCutoff)
		{
			//cout << "Filtering " << object.name << " because it's " << chrono::duration<double>(Clock::now() - object.LastSeen).count() << "s old" << endl;
			continue;
		}
		auto obj = object.ToGLObject();
		if (!obj.has_value())
		{
			continue;
		}
		outobj.push_back(obj.value());
		vector<GLObject> childs = ObjectData::ToGLObjects(object.Childs);
		for (size_t i = 0; i < childs.size(); i++)
		{
			childs[i].location = obj.value().location * childs[i].location; //apply parent transform to child
		}
		outobj.insert(outobj.end(), childs.begin(), childs.end());
	}
	return outobj;
}