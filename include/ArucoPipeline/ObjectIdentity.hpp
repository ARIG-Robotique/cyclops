#pragma once

#include <cstdint>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <opencv2/core/affine.hpp>

enum class CDFRTeam
{
	Unknown,
	Yellow,
	Blue
};


const std::map<CDFRTeam, std::string> TeamNames = {
	{CDFRTeam::Unknown, "UNKNOWN"},
	{CDFRTeam::Blue, "BLUE"},
	{CDFRTeam::Yellow, "YELLOW"}
};

std::ostream& operator << (std::ostream& out, CDFRTeam Team);

enum class ObjectType
{
	Unknown,
	Tag,
	ReferenceAbsolute,
	ReferenceRelative,
	Camera,
	Object,
	Robot,
	TopTracker,
	TeamTracker,

	SolarPanel,
	Team
};

const std::map<ObjectType, std::string> ObjectTypeNames =
{
	{ObjectType::Unknown, 			"UNKNOWN"},
	{ObjectType::Tag, 				"TAG"},
	{ObjectType::ReferenceAbsolute, "REFERENCE_ABSOLUTE"},
	{ObjectType::ReferenceRelative, "REFERENCE_RELATIVE"},
	{ObjectType::Camera, 			"CAMERA"},
	{ObjectType::Object, 			"OBJECT"},
	{ObjectType::Robot, 			"ROBOT"},
	{ObjectType::TopTracker, 		"TOP_TRACKER"},
	{ObjectType::TeamTracker, 		"TEAM_TRACKER"},

	{ObjectType::SolarPanel, 		"SOLAR_PANEL"},
	{ObjectType::Team, 				"TEAM"}
};

std::ostream& operator << (std::ostream& out, ObjectType Type);

struct ObjectData
{
	ObjectType type;
	std::string name;
	std::string metadata;
	cv::Affine3d location;

	std::vector<ObjectData> Childs;

	ObjectData(ObjectType InType = ObjectType::Unknown, const std::string InName = "None", cv::Affine3d InLocation = cv::Affine3d::Identity())
		:type(InType), name(InName), location(InLocation)
	{}

	ObjectData(ObjectType InType, const std::string InName, double posX, double posY, double posZ)
		:type(InType), name(InName)
	{
		location = cv::Affine3d::Identity();
		location.translation(cv::Vec3d(posX, posY, posZ));
	}

	std::optional<struct GLObject> ToGLObject() const;

	static std::vector<GLObject> ToGLObjects(const std::vector<ObjectData>& data);
};
